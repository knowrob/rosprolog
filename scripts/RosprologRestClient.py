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
		self.finished = False
		self.solutions = []
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
			self.finished = False
			self.solutions = []
			return True

	def get_next_solution(self):
		next_solution = self._next_solution_srv(id=str(self.id))
		rospy.loginfo(str(next_solution))
		if not next_solution.status == PrologNextSolutionResponse.OK:
			return jsonify(success=False)
		else:
			solution = json.loads(next_solution.solution)
			self.solutions.append(solution)
			return solution

	def get_all_solutions(self):
		try:
			while not self.finished:
				next_solution = self._next_solution_srv(id=str(self.id))
				if next_solution.status == PrologNextSolutionResponse.OK:
					self.solutions.append(json.loads(next_solution.solution))
				elif next_solution.status == PrologNextSolutionResponse.NO_SOLUTION:
					break
				else:
					return jsonify(success=False)
		finally:
			self.finish()
		return self.solutions

	def finish(self):
		self._finish_query_srv(id=str(self.id))
		self.finished = True