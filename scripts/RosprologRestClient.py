import rospy
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologNextSolutionResponse, PrologFinish
import json
from flask import Flask, jsonify, request


class RosprologRestClient:
	def __init__(self, name_space='rosprolog', timeout=None, wait_for_services=True):
		"""
		:type name_space: str
		:param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
		:type timeout: int
		"""
		self.id = 0
		self.query_ids = []
		self._simple_query_srv = rospy.ServiceProxy(
			'{}/query'.format(name_space), PrologQuery)
		self._next_solution_srv = rospy.ServiceProxy(
			'{}/next_solution'.format(name_space), PrologNextSolution)
		self._finish_query_srv = rospy.ServiceProxy(
			'{}/finish'.format(name_space), PrologFinish)
		if wait_for_services:
			rospy.loginfo('waiting for {} services'.format(name_space))
			self._finish_query_srv.wait_for_service(timeout=timeout)
			self._simple_query_srv.wait_for_service(timeout=timeout)
			self._next_solution_srv.wait_for_service(timeout=timeout)
			rospy.loginfo('{} services ready'.format(name_space))

	def post_query(self, query):
		self.id += 1
		result = self._simple_query_srv(id=str(self.id), query=query)
		if not result.ok:
			self.id -= 1
			return False
		else:
			self.query_ids.append(str(self.id))
			return True

	def get_next_solution(self, query_id):
		next_solution = self._next_solution_srv(id=query_id)
		if not next_solution.status == PrologNextSolutionResponse.OK:
			return jsonify(success=False)
		else:
			solution = json.loads(next_solution.solution)
			return solution

	def get_all_next_solutions(self, query_id):
		solutions = []
		try:
			while query_id in self.query_ids:
				next_solution = self._next_solution_srv(id=query_id)
				if next_solution.status == PrologNextSolutionResponse.OK:
					solutions.append(json.loads(next_solution.solution))
				elif next_solution.status == PrologNextSolutionResponse.NO_SOLUTION:
					break
				else:
					return jsonify(success=False)
		finally:
			self.finish_query(query_id)
		return solutions

	def finish_query(self, query_id):
		if query_id in self.query_ids:
			self._finish_query_srv(id=query_id)
			self.query_ids.remove(query_id)
			return jsonify(success=True)
		else:
			return jsonify(success=False)