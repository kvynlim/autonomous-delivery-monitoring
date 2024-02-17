from aiohttp import ClientSession
from asyncio import create_task, gather, run


async def __send_request(session: ClientSession, url: str):
    async with session.get(url) as response:
        return await response.json()


async def test_all_robots():
    async with ClientSession() as session:
        result = await __send_request(session, "http://0.0.0.0:5000/api/robots")
        print("Test all robots API --", result, "\n")


async def test_robot():
    async with ClientSession() as session:
        tasks = []
        for url in [
            "http://0.0.0.0:5000/api/robots/tb3_0",
            "http://0.0.0.0:5000/api/robots/tb3_1",
            "http://0.0.0.0:5000/api/robots/tb3_2",
        ]:
            tasks.append(create_task(__send_request(session, url)))

        for result in await gather(*tasks):
            print("Test robot API -- ", result, "\n")


if __name__ == "__main__":
    while True:
        run(test_all_robots())
        run(test_robot())
        print(
            "############################################################################################################################################################"
        )
