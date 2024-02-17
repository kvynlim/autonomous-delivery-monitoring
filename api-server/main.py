from fastapi import FastAPI
from os.path import dirname, join, realpath
from sys import path
from uvicorn import run

current_path = realpath(__file__)
path.append(join(dirname(current_path), "src"))
path.append(join(dirname(dirname(current_path)), "lib", "src"))

from src.fleet_controller import FleetController

app = FastAPI()
app.include_router(FleetController().get_router())


def main():
    run("main:app", host="0.0.0.0", port=5000, log_level="info")


if __name__ == "__main__":
    main()
