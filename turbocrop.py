# -*- coding: utf-8 -*-
# file: turbocrop
# author: Sean O'Brien
# date: 5/21/2019
# description: For each measurement series in the project, the
#  most likely base plane is discovered via grouping of like normals.
#  A crop operation is applied to everything below the base plane.
#  After all series' base planes have been discovered in this manner,
#  each series is transposed onto the initial series using the reference
#  points that lie above the base plane
# ------------------------------------------------------------------------------

# -- Imports
import gom

from numpy import (concatenate, array, dot, cross, subtract, setdiff1d, flip)
from pyntcloud import PyntCloud
from pandas import DataFrame

# drawCircles
def drawCircles(points, indices):
	if len(indices) == 0:
		return
		
	circles = []	
	for index in indices:
		circles.append(gom.script.primitive.create_circle_by_point_normal_radius(
		point = gom.Vec3d(*points.data.coordinate[0][index]),
		normal = gom.Vec3d(*points.data.normal[0][index]),
		radius = 5))
	
	gom.script.sys.edit_properties (data = circles,
		label_grouping_enabled = False, 
		label_show = False)
	
	gom.script.inspection.measure_by_no_measuring_principle ( elements = circles )
	return circles

def drawPlane(points, indices, size = 200):
	if len(indices) == 0:
		return

	if len(indices) < 3:
		plane = gom.script.primitive.create_plane_by_point_normal(
			point = gom.Vec3d(*points.data.coordinate[0][indices[0]]),
			normal = gom.Vec3d(*points.data.normal[0][indices[0]]))
	else:
		plane = gom.script.primitive.create_plane_by_3_points(
			point1 = gom.Vec3d(*points.data.coordinate[0][indices[0]]),
			point2 = gom.Vec3d(*points.data.coordinate[0][indices[1]]),
			point3 = gom.Vec3d(*points.data.coordinate[0][indices[-1]]))

	gom.script.sys.edit_properties(
		data = plane,
		label_grouping_enabled = False,
		label_show = False,
		elem_use_calc_size = False,
		plane_draw_length = size,
		plane_draw_width = size)
		
	gom.script.inspection.measure_by_no_measuring_principle(elements = [plane])
	return plane

def partitionPoints(points, plane):
	above = []
	below = []
	
	planePoint1 = points.data.coordinate[0][plane[0]]
	planePoint2 = points.data.coordinate[0][plane[-1]]
	planePoint3 = points.data.coordinate[0][plane[1]]
	
	planeNormal = cross(
		subtract(planePoint1, planePoint2),
		subtract(planePoint3, planePoint2)
	)

	if planeNormal[1] < 0:
		planeNormal = planeNormal * -1

	indices = setdiff1d(array(range(0, len(points.data.coordinate[0]))), plane)
	for index in indices:
		pointVector = subtract(points.data.coordinate[0][index], planePoint1)
		product = dot(pointVector, planeNormal)
		if product > 0:
			above.append(index)
		elif product < 0:
			below.append(index)
			
	return {'above': above, 'below': below}
	
def findPlane(points):
	indices = array([range(0, len(points.data.coordinate[0]))])
	pointsWithIndices = concatenate((indices.T, points.data.coordinate[0]), axis = 1)
	cloud = PyntCloud(DataFrame(pointsWithIndices, columns = ['index', 'x', 'y', 'z']))
	
	cloud.add_scalar_field('plane_fit', max_dist=1)
	plane = cloud.points.query('is_plane == 1').to_numpy()
	
	return plane.T[0].astype(int)

# extractPointsByIndex
#   inputs:
#     points - points object from which to extract points
#     indices - array of indices to extract
#     normals [false] - extract the normals instead of the coordinates
#
#   output:
#     data - array of either points or normals
def extractPointsByIndex(points, indices, normals = False):
	out = []
	keyword = 'normal' if normals else 'coordinate'
	for i in indices:
		out.append(points.get('%s[%d]' % (keyword, i)))
		
	return out

def cropByPlane(series, points, indices):
	point1 = {'normal': gom.Vec3d(*points.data.normal[0][indices[0]]),
			'point': gom.Vec3d(*points.data.coordinate[0][indices[0]])}
	point2 = {'normal': gom.Vec3d(*points.data.normal[0][indices[-1]]),
			'point': gom.Vec3d(*points.data.coordinate[0][indices[-1]])}
	point3 = {'normal': gom.Vec3d(*points.data.normal[0][indices[1]]),
			'point': gom.Vec3d(*points.data.coordinate[0][indices[1]])}

	gom.script.atos.cut_out_points (
		cut_out_points_below_plane=True, 
		measurement_series=[series], 
		plane_position=2, 
		point1=point1, 
		point2=point2, 
		point3=point3)

for index, series in enumerate(gom.app.project.measurement_series):
#series = gom.app.project.measurement_series[0]
	points = series.results['points']
	plane = findPlane(points)
	planeElement = drawPlane(points = points, indices = plane, size = 1000)
	
	gom.script.view.wait_for_rendering(True)
	time.sleep(1)

	cropByPlane(series, points, plane)

	if index > 0:
		part = partitionPoints(points, plane)
		circles = drawCircles(points, part['above'])
		gom.script.view.wait_for_rendering(True)
		time.sleep(1)

		gom.script.atos.transform_by_common_reference_points (
			reference_measurement_series=gom.app.project.measurement_series[0],
			source_measurement_series=series,
			transformation_points=extractPointsByIndex(points, part['above']))
		gom.script.view.wait_for_rendering(True)
		gom.script.cad.delete_element(elements = circles)

	gom.script.cad.delete_element(elements = [planeElement])