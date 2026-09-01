FROM python:3.10-slim

RUN useradd --create-home --uid 1000 --user-group --shell /usr/sbin/nologin foxglove

WORKDIR /app
COPY serve_assets.py /app/serve_assets.py

USER foxglove
CMD ["python3", "/app/serve_assets.py", "--directory", "/assets"]
