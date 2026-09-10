from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from .config import load_config
from .server import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV tracking preview and guarded ground test")
    parser.add_argument(
        "--config",
        default=str(Path.cwd() / "config.toml"),
        help="Path to config.toml",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    app = create_app(config)
    uvicorn.run(
        app,
        host=config.server.host,
        port=config.server.port,
        log_level="info",
        access_log=False,
    )


if __name__ == "__main__":
    main()
