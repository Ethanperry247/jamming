from fastapi import FastAPI, Request, HTTPException, Depends, status
from .database import database
from pydantic import BaseModel
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles

import http



class User(BaseModel):
    username: str
    password: str

class AuthUser(User):
    id: str

app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")

@app.on_event("startup")
async def start():
    await database.startup()

@app.on_event("shutdown")
async def stop():
    await database.shutdown()

# Simple login example
@app.post("/login", status_code=200)
async def root(user: User, db=Depends(database.provide_connection)):
    valid_user = await db.fetch_one(query="SELECT * FROM users WHERE username=:username AND password = crypt(:password, password)", values={'username': user.username, 'password': user.password})
    if valid_user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="NOT_AUTHORIZED"
        )
    else:
        return str(dict(valid_user)['id'])

@app.post("/signup", status_code=200)
async def root(user: User, db=Depends(database.provide_connection)):
    # print(await request.json())
    user_exists = await db.fetch_one("SELECT * FROM users WHERE username=:username", values={'username': user.username})
    if (user_exists):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="ALREADY_EXISTS"
        )
    async with db.transaction():
        await db.execute("INSERT INTO users (username, password) VALUES (:username, crypt(:password, gen_salt('md5')))", values={'username': user.username, 'password': user.password})
    

# Simple authorized resource example
@app.put("/resource", status_code=200)
async def root(user: AuthUser, db=Depends(database.provide_connection)):
    raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Resource not found.",
            )

# Simple HTML example
@app.get("/", response_class=HTMLResponse)
async def read_items():
    html_content = f"""
    <html>
        <head>
            <title>Some HTML in here</title>
        </head>
        <body>
            <h1>Look! HTML!</h1>
        </body>
    </html>
    """
    return HTMLResponse(content=html_content, status_code=200)

# Templated HTML examples
@app.get("/items", response_class=HTMLResponse)
async def read_item(request: Request):
    return templates.TemplateResponse("index.html", {"request": request, "id": "1234"})

@app.get("/signup", response_class=HTMLResponse)
async def read_item(request: Request):
    return templates.TemplateResponse("login.html", {"request": request})

# Simple json example
@app.get("/json", status_code=200)
async def root(request: Request):
    return {"Hello": "World"}

# @app.get("/database", status_code=200)
# async def database(request: Request, db=Depends(database.provide_connection)):
#     info = await db.fetch_all("SELECT * FROM users")
#     return info