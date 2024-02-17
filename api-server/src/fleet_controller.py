from domain.fleet_manager.fleet_manager import FleetManager
from fastapi import APIRouter, Request, status
from fastapi.responses import JSONResponse
from http import HTTPStatus
from infrastructure.utility.logger import Logger


class FleetController:
    def __init__(self):
        self.__router = APIRouter(tags=[self.__class__.__name__])
        self.__router.add_api_route(
            path="/api/robots",
            endpoint=self.__query_robots,
            methods=["GET"],
            responses={
                HTTPStatus.OK.value: {"description": f"{HTTPStatus.OK._name_}"},
                HTTPStatus.BAD_REQUEST.value: {"description": f"{HTTPStatus.BAD_REQUEST._name_}"},
            },
        )
        self.__router.add_api_route(
            path="/api/robots/{robot_id}",
            endpoint=self.__query_robot,
            methods=["GET"],
            responses={
                HTTPStatus.OK.value: {"description": f"{HTTPStatus.OK._name_}"},
                HTTPStatus.BAD_REQUEST.value: {"description": f"{HTTPStatus.BAD_REQUEST._name_}"},
            },
        )
        self.__fleet_manager = FleetManager()

    def get_router(self):
        return self.__router

    async def __query_robots(self, request: Request):
        """
        **Query robots status**

        Returns a list of all available robots along with their current location and battery level.
        """
        Logger().debug(f"FleetController -- {request.client.host} query robots")
        result = await self.__fleet_manager.get_robot_status()
        if isinstance(result, dict):
            return JSONResponse(content=result, status_code=status.HTTP_200_OK)
        else:
            return JSONResponse(content=None, status_code=status.HTTP_400_BAD_REQUEST)

    async def __query_robot(self, robot_id: str, request: Request):
        """
        **Query robot status by robot id**

        Returns detailed information about a specific robot identified by robot_id, including its status and battery level.
        """
        Logger().debug(f"FleetController -- {request.client.host} query robot {robot_id}")
        result = await self.__fleet_manager.get_robot_status(robot_id)
        if isinstance(result, dict):
            return JSONResponse(content=result, status_code=status.HTTP_200_OK)
        else:
            return JSONResponse(content=None, status_code=status.HTTP_400_BAD_REQUEST)
