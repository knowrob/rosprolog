#!/usr/bin/env python3
import rospy
import json
from RosprologRestClient import RosprologRestClient
from gevent.pywsgi import WSGIServer
from flask import Flask
from flask_restplus import Api, Resource, fields

app = Flask(__name__)
app.config['RESTPLUS_MASK_SWAGGER'] = False

api = Api(app, version='1.0', title='KnowRob API',
          description='KnowRob API reference',
          )

query = api.model('Query', {
	'id': fields.Integer(readonly=True, description='The query unique identifier'),
	'query': fields.String(required=True, description='The query string')
})

ns = api.namespace('knowrob/api/v1.0',
                   description='Operations related to KnowRob')

rosrest = RosprologRestClient()

@ns.route("/solutions/getNextSolution/<string:query_id>")
class NextSolutions(Resource):
	def get(self, query_id):
		return rosrest.get_next_solution(query_id)

@ns.route("/solutions/getAllNextSolutions/<string:query_id>")
class AllNextSolution(Resource):
	def get(self, query_id):
		return rosrest.get_all_next_solutions(query_id)

@ns.route("/query")
class Query(Resource):
	@ns.expect(query)
	@ns.marshal_with(query)
	def post(self):
		rosrest.post_query(api.payload['query'])
		api.payload['id'] = rosrest.id
		return api.payload

@ns.route("/query/<string:query_id>")
class DeleteQuery(Resource):
	def delete(self, query_id):
		return rosrest.finish_query(query_id)

if __name__ == '__main__':
	rospy.init_node('rosprolog_rest', anonymous=True)
	http_server = WSGIServer(('', 62226), app)
	http_server.serve_forever()