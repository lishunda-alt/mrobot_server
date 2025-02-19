from flask import Flask, request
from dependent_library import PriorityQueue
from asyncioServer import tcpServer, main
from threading import Thread
import json
import asyncio
from _ext import *
app = Flask(__name__)
pq = PriorityQueue()


@app.route('/open_charge', methods=['POST'])
def open_charge():
    post_data = json.loads(request.data)
    target_degree = post_data.get("target_degree")
    tcpServer.task["openCharge"]["status"] = True
    print("acquire lock")
    tcpServer.startChargeSemaphore.acquire()
    print(tcpServer.task["openCharge"])
    return make_return(_code=200, _message="OK")


@app.route('/close_charge', methods=['GET'])
def close_charge():
    tcpServer.task["closeCharge"]["status"] = True
    print("acquire lock")
    tcpServer.stopChargeSemaphore.acquire()
    print(tcpServer.task["closeCharge"])
    return make_return(_code=200, _message="OK")

Thread(target=app.run, args=("0.0.0.0", 8080)).start()
# app.run("0.0.0.0", 8080)
asyncio.run(main())