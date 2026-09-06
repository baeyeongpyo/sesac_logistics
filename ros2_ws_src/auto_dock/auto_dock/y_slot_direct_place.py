"""Simple post-staging policy for automatic Y PLACE commands."""


def post_staging_action(insertion_enabled):
    """At the 40 cm staging endpoint, insert or hold without looking again."""
    return "INSERT" if insertion_enabled else "HOLD"
