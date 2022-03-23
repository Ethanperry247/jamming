import databases
import sqlalchemy
import os

DATABASE_URL = os.environ.get('DATABASE_URL')
DATABASE_URL = 'postgresql://postgres:postgres@0.0.0.0:5432/dev' if DATABASE_URL is None else DATABASE_URL
database = databases.Database(DATABASE_URL)

async def startup():
    # Start database on app startup.
    try:
        await database.connect()
        print("INFO:     Successfully connected to database.")
    except ConnectionRefusedError:
        print("ERROR:    Could not connect to database.")
        raise 

async def shutdown():
    # Stop database on shutdown.
    print("INFO:     Disconnecting from database.")
    await database.disconnect()

def provide_connection() -> databases.Database:
        return database