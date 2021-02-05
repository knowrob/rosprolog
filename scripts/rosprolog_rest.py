#!/usr/bin/env python
import re
import os
import traceback

from builtins import input

import rospy
import readline
import sys
import json
from rosprolog_client import PrologException, Prolog, PrologQuery
from json_prolog_msgs import srv

from flask import Flask, jsonify, request
from flask_restful import reqparse, abort, Api, Resource

app = Flask(__name__)
# api = Api(app)

parser = reqparse.RequestParser()
parser.add_argument('query', type=str)
parser.add_argument('id', type=int)

class RosRest(Resource):
    def __init__(self, name_space='rosprolog', timeout=None, wait_for_services=True):
        """
        :type name_space: str
        :param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        :type timeout: int
        """
        self._simple_query_srv = rospy.ServiceProxy('{}/query'.format(name_space), srv.PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy('{}/next_solution'.format(name_space), srv.PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy('{}/finish'.format(name_space), srv.PrologFinish)
        if wait_for_services:
            rospy.loginfo('waiting for {} services'.format(name_space))
            self._finish_query_srv.wait_for_service(timeout=timeout)
            self._simple_query_srv.wait_for_service(timeout=timeout)
            self._next_solution_srv.wait_for_service(timeout=timeout)
            rospy.loginfo('{} services ready'.format(name_space))

    @app.route('/knowrob/api/v1.0/query', methods=['POST'])
    def query(self):
        request.get_json(force=True)
        args = parser.parse_args()
        query_str = str(args['query'])
        in_id = str(args['id'])
        result = self._simple_query_srv(id=in_id, query=query_str, mode=1)
        if not result.ok:
            return jsonify(sucess=False)
        return jsonify(sucess=True)

    @app.route('/knowrob/api/v1.0/next_solution', methods=['GET'])
    def next(self):
        request.get_json(force=True)
        args = parser.parse_args()
        in_id = str(args['id'])
        next_solution = self._next_solution_srv(id=self.get_id())
        if not result.ok:
            return jsonify(sucess=False)
        return json.loads(next_solution)

    @app.route('/knowrob/api/v1.0/all_solutions', methods=['GET'])
    def all(self):
        request.get_json(force=True)
        args = parser.parse_args()
        in_id = str(args['id'])
        solutions = list()
        finished = False
        try:
            while not finished:
                next_solution = self._next_solution_srv(id=self.get_id())
                if next_solution.status == srv.PrologNextSolutionResponse.OK:
                    solutions.append(json.loads(next_solution.solution))
                elif next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION:
                    break
        finally:
            self.finish()
        if not result.ok:
            return jsonify(sucess=False)
        return json.loads(solutions)

    @app.route('/knowrob/api/v1.0/all_solutions', methods=['POST'])
    def finish(self):
        request.get_json(force=True)
        args = parser.parse_args()
        in_id = str(args['id'])
        if not self._finished:
            try:
                self._finish_query_srv(id=in_id)
            finally:
                self._finished = True


if __name__ == '__main__':
    rospy.init_node('rosprolog_rest', anonymous=True)
    rosrest = RosRest()
    app.run(host='0.0.0.0',debug=True,port=62226)
