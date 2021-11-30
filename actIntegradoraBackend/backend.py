import flask
import os
from flask.json import jsonify
import uuid
from actIntegradora import Robot, Pila, Almacen

games = {}

app = flask.Flask(__name__)

@app.route("/games", methods=["POST"])
def create():
    global games
    id = str(uuid.uuid4())
    games[id] = Almacen()
    return "ok", 201, {'Location': f"/games/{id}", 'nRobots': f"{games[id].nRobots}", 'nCajas': f"{games[id].nCajas}"}

@app.route("/games/<id>", methods=["GET"])
def queryStateCars(id):
    global model
    model = games[id]
    model.step()
    agents = model.schedule.agents
    
    listRobots = []
    listPilas = []

    for agent in agents:
        if(isinstance(agent, Robot)):
            listRobots.append({"id": agent.unique_id,"x": agent.pos[0], "y": agent.pos[1]})
        elif(isinstance(agent, Pila)):
            listPilas.append({"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1], "nCajas": agent.nCajas, "cargada": agent.cargada})

    return jsonify({"robots": listRobots, "pilas": listPilas})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=int(os.environ.get('PORT', 5001)))