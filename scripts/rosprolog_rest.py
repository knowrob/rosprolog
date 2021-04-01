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
model = {}
model["123"] = "abc"
query = api.model('Query', {
	'id': fields.Integer(readonly=True, description='The query unique identifier'),
	'query': fields.String(required=True, description='The query string'),
	'solutionCount': fields.Integer(required=True, default=100, description='The number of solutions'),
	'response': fields.Raw(readonly=True, description='The response dictionary')
})

ns = api.namespace('knowrob/api/v1.0',
                   description='Operations related to KnowRob')

rosrest = RosprologRestClient()

@ns.route("/query")
class Query(Resource):
	@ns.expect(query)
	@ns.marshal_with(query)
	def post(self):
		rosrest.post_query(api.payload['query'])
		api.payload['response'] = rosrest.get_solutions(api.payload['solutionCount'])
		return api.payload

if __name__ == '__main__':
	rospy.init_node('rosprolog_rest', anonymous=True)
	http_server = WSGIServer(('', 62226), app)
	http_server.serve_forever()