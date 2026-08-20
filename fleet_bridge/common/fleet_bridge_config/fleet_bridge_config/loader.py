from collections.abc import Mapping
from pathlib import Path
import re
from types import MappingProxyType

import yaml

from .models import (
    CriticalConfig,
    FilterConfig,
    FleetConfig,
    QosConfig,
    RateConfig,
    ServerConfig,
    TopicConfig,
    VehicleConfig,
)


ENVIRONMENT_PATTERN = re.compile(r'\$\{([A-Z_][A-Z0-9_]*)\}')
IDENTIFIER_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
MESSAGE_TYPE_PATTERN = re.compile(
    r'^[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$',
)


class ConfigError(ValueError):
    """Raised when a fleet bridge configuration is unsafe or ambiguous."""


def _read_yaml(path: Path | str) -> object:
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except OSError as error:
        raise ConfigError(f'cannot read config {path}: {error}') from error
    except yaml.YAMLError as error:
        raise ConfigError(f'invalid YAML in {path}: {error}') from error


def _mapping(value: object, location: str) -> dict:
    if not isinstance(value, Mapping):
        raise ConfigError(f'{location} must be a mapping')
    return dict(value)


def _list(value: object, location: str) -> list:
    if not isinstance(value, list):
        raise ConfigError(f'{location} must be a list')
    return value


def _keys(value: dict, allowed: set[str], location: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ConfigError(f'{location} has unknown keys: {", ".join(unknown)}')


def _required(value: dict, names: set[str], location: str) -> None:
    missing = sorted(names - set(value))
    if missing:
        raise ConfigError(f'{location} missing keys: {", ".join(missing)}')


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f'{location} must be an integer')
    if value < minimum or value > maximum:
        raise ConfigError(f'{location} must be between {minimum} and {maximum}')
    return value


def _positive_number(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f'{location} must be a number')
    number = float(value)
    if number <= 0:
        raise ConfigError(f'{location} must be greater than zero')
    return number


def _boolean(value: object, location: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f'{location} must be a boolean')
    return value


def _string(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigError(f'{location} must be a non-empty string')
    return value


def _identifier(value: object, location: str) -> str:
    identifier = _string(value, location)
    if not IDENTIFIER_PATTERN.fullmatch(identifier):
        raise ConfigError(f'{location} must be an identifier')
    return identifier


def _topic_name(value: object, location: str, robot_id: str) -> str:
    topic = _string(value, location).replace('{robot}', robot_id)
    if '{' in topic or '}' in topic:
        raise ConfigError(f'{location} contains an unsupported template')
    if not topic.startswith('/') or topic == '/' or '//' in topic or topic.endswith('/'):
        raise ConfigError(f'{location} must be an absolute ROS topic')
    return topic


def _namespace(value: object, location: str) -> str:
    namespace = _string(value, location)
    if not namespace.startswith('/') or namespace == '/' or namespace.endswith('/'):
        raise ConfigError(f'{location} must be an absolute non-root namespace')
    return namespace


def _expand_environment(value: object, environ: Mapping[str, str]) -> object:
    if isinstance(value, str):
        def replace(match: re.Match) -> str:
            name = match.group(1)
            if name not in environ or not environ[name]:
                raise ConfigError(f'missing environment value: {name}')
            return environ[name]

        return ENVIRONMENT_PATTERN.sub(replace, value)
    if isinstance(value, list):
        return [_expand_environment(item, environ) for item in value]
    if isinstance(value, Mapping):
        return {
            key: _expand_environment(item, environ)
            for key, item in value.items()
        }
    return value


def load_fleet(path: Path | str, environ: Mapping[str, str]) -> FleetConfig:
    document = _mapping(_expand_environment(_read_yaml(path), environ), 'fleet')
    _keys(document, {'server', 'vehicles'}, 'fleet')
    _required(document, {'server', 'vehicles'}, 'fleet')

    raw_server = _mapping(document['server'], 'fleet.server')
    _keys(raw_server, {'domain_id', 'foxglove_port'}, 'fleet.server')
    _required(raw_server, {'domain_id', 'foxglove_port'}, 'fleet.server')
    server = ServerConfig(
        domain_id=_integer(raw_server['domain_id'], 'server.domain_id', 0, 232),
        foxglove_port=_integer(raw_server['foxglove_port'], 'server.foxglove_port', 1, 65535),
    )

    vehicles = []
    ids = set()
    domains = set()
    for index, raw_value in enumerate(_list(document['vehicles'], 'fleet.vehicles')):
        location = f'fleet.vehicles[{index}]'
        raw = _mapping(raw_value, location)
        allowed = {'id', 'domain_id', 'foxglove_uri', 'namespace', 'enabled'}
        _keys(raw, allowed, location)
        _required(raw, allowed, location)
        vehicle = VehicleConfig(
            id=_identifier(raw['id'], f'{location}.id'),
            domain_id=_integer(raw['domain_id'], f'{location}.domain_id', 0, 232),
            foxglove_uri=_string(raw['foxglove_uri'], f'{location}.foxglove_uri'),
            namespace=_namespace(raw['namespace'], f'{location}.namespace'),
            enabled=_boolean(raw['enabled'], f'{location}.enabled'),
        )
        if not vehicle.foxglove_uri.startswith(('ws://', 'wss://')):
            raise ConfigError(f'{location}.foxglove_uri must use ws:// or wss://')
        if vehicle.id in ids:
            raise ConfigError(f'duplicate vehicle id: {vehicle.id}')
        if vehicle.domain_id in domains:
            raise ConfigError(f'duplicate vehicle domain_id: {vehicle.domain_id}')
        ids.add(vehicle.id)
        domains.add(vehicle.domain_id)
        vehicles.append(vehicle)

    if not vehicles:
        raise ConfigError('fleet.vehicles must not be empty')
    return FleetConfig(server=server, vehicles=tuple(vehicles))


def _load_qos(raw_value: object, location: str) -> QosConfig:
    raw = _mapping(raw_value, location)
    allowed = {'reliability', 'durability', 'history', 'depth'}
    _keys(raw, allowed, location)
    _required(raw, allowed, location)
    reliability = _string(raw['reliability'], f'{location}.reliability')
    durability = _string(raw['durability'], f'{location}.durability')
    history = _string(raw['history'], f'{location}.history')
    if reliability not in {'best_effort', 'reliable'}:
        raise ConfigError(f'{location}.reliability is invalid')
    if durability not in {'volatile', 'transient_local'}:
        raise ConfigError(f'{location}.durability is invalid')
    if history != 'keep_last':
        raise ConfigError(f'{location}.history must be keep_last')
    return QosConfig(
        reliability=reliability,
        durability=durability,
        history=history,
        depth=_integer(raw['depth'], f'{location}.depth', 1, 1000),
    )


def _load_rate(raw_value: object, location: str) -> RateConfig:
    raw = _mapping(raw_value, location)
    _keys(raw, {'max_rate_hz'}, location)
    if 'max_rate_hz' not in raw:
        return RateConfig()
    return RateConfig(
        max_rate_hz=_positive_number(raw['max_rate_hz'], f'{location}.max_rate_hz'),
    )


def _load_filter(raw_value: object, location: str) -> FilterConfig:
    raw = _mapping(raw_value, location)
    allowed = {'mode', 'max_rate_hz', 'heartbeat_sec', 'thresholds', 'critical'}
    _keys(raw, allowed, location)
    _required(raw, {'mode'}, location)
    mode = _string(raw['mode'], f'{location}.mode')
    if mode not in {'passthrough', 'rate', 'on_change'}:
        raise ConfigError(f'{location}.mode is invalid')

    max_rate_hz = None
    if 'max_rate_hz' in raw:
        max_rate_hz = _positive_number(raw['max_rate_hz'], f'{location}.max_rate_hz')
    if mode in {'rate', 'on_change'} and max_rate_hz is None:
        raise ConfigError(f'{location}.max_rate_hz is required for {mode}')

    heartbeat_sec = None
    if 'heartbeat_sec' in raw:
        heartbeat_sec = _positive_number(raw['heartbeat_sec'], f'{location}.heartbeat_sec')

    raw_thresholds = _mapping(raw.get('thresholds', {}), f'{location}.thresholds')
    thresholds = {}
    for name, value in raw_thresholds.items():
        field_name = _string(name, f'{location}.thresholds field')
        thresholds[field_name] = _positive_number(
            value,
            f'{location}.thresholds.{field_name}',
        )
    if mode == 'on_change' and not thresholds:
        raise ConfigError(f'{location}.thresholds is required for on_change')

    raw_critical = _mapping(raw.get('critical', {}), f'{location}.critical')
    _keys(raw_critical, {'field', 'below', 'bypass_rate_limit'}, f'{location}.critical')
    critical = CriticalConfig()
    if raw_critical:
        _required(
            raw_critical,
            {'field', 'below', 'bypass_rate_limit'},
            f'{location}.critical',
        )
        below = raw_critical['below']
        if isinstance(below, bool) or not isinstance(below, (int, float)):
            raise ConfigError(f'{location}.critical.below must be a number')
        critical = CriticalConfig(
            field=_string(raw_critical['field'], f'{location}.critical.field'),
            below=float(below),
            bypass_rate_limit=_boolean(
                raw_critical['bypass_rate_limit'],
                f'{location}.critical.bypass_rate_limit',
            ),
        )

    if mode == 'passthrough' and set(raw) != {'mode'}:
        raise ConfigError(f'{location} passthrough does not accept filter options')
    return FilterConfig(
        mode=mode,
        max_rate_hz=max_rate_hz,
        heartbeat_sec=heartbeat_sec,
        thresholds=MappingProxyType(thresholds),
        critical=critical,
    )


def load_telemetry(path: Path | str, robot_id: str) -> tuple[TopicConfig, ...]:
    robot_id = _identifier(robot_id, 'robot_id')
    document = _mapping(_read_yaml(path), 'telemetry')
    _keys(document, {'version', 'topics'}, 'telemetry')
    _required(document, {'version', 'topics'}, 'telemetry')
    if document['version'] != 1:
        raise ConfigError('telemetry.version must be 1')

    topics = []
    ids = set()
    active_uplinks = set()
    active_targets = set()
    for index, raw_value in enumerate(_list(document['topics'], 'telemetry.topics')):
        location = f'telemetry.topics[{index}]'
        raw = _mapping(raw_value, location)
        allowed = {
            'id', 'enabled', 'source', 'uplink', 'target', 'type',
            'filter', 'worker_rate', 'qos', 'debug',
        }
        _keys(raw, allowed, location)
        _required(raw, allowed, location)
        message_type = _string(raw['type'], f'{location}.type')
        if not MESSAGE_TYPE_PATTERN.fullmatch(message_type):
            raise ConfigError(f'{location}.type is not a valid message type')
        topic = TopicConfig(
            id=_identifier(raw['id'], f'{location}.id'),
            enabled=_boolean(raw['enabled'], f'{location}.enabled'),
            source=_topic_name(raw['source'], f'{location}.source', robot_id),
            uplink=_topic_name(raw['uplink'], f'{location}.uplink', robot_id),
            target=_topic_name(raw['target'], f'{location}.target', robot_id),
            message_type=message_type,
            filter=_load_filter(raw['filter'], f'{location}.filter'),
            worker_rate=_load_rate(raw['worker_rate'], f'{location}.worker_rate'),
            qos=_load_qos(raw['qos'], f'{location}.qos'),
            debug=_boolean(raw['debug'], f'{location}.debug'),
        )
        if topic.id in ids:
            raise ConfigError(f'duplicate topic id: {topic.id}')
        ids.add(topic.id)
        if topic.filter.mode == 'passthrough' and topic.source != topic.uplink:
            raise ConfigError(f'{location}.uplink must equal source for passthrough')
        if topic.filter.mode != 'passthrough' and topic.source == topic.uplink:
            raise ConfigError(f'{location}.uplink must differ from filtered source')
        if topic.enabled:
            if topic.uplink in active_uplinks:
                raise ConfigError(f'duplicate uplink: {topic.uplink}')
            if topic.target in active_targets:
                raise ConfigError(f'duplicate target: {topic.target}')
            active_uplinks.add(topic.uplink)
            active_targets.add(topic.target)
        topics.append(topic)

    if not topics:
        raise ConfigError('telemetry.topics must not be empty')
    return tuple(topics)

