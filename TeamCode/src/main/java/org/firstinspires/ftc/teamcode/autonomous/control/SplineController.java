package org.firstinspires.ftc.teamcode.autonomous.control;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class SplineController
{
	// Returns the position as a point on the curve with the given value t, between 0 and 1, 0 being the beginning of the curve, 1 being the end.
	// The function is split into 4 distinct terms, one for each point. Each term represents the component vector created by that point.
	// The sum of all these vectors returns the vector which describes the position based on t. The path of the vector over all values t between 0 and 1 describes the spline.

	private Position getPositionVector(Position p0, Position p1, Position p2, Position p3, double t)
	{
		double xVector =	( p0.x * ((1-t) * (1-t) * (1-t)) ) +
							( p1.x * ((1-t) * (1-t)) * t ) +
							( p2.x * (1-t) * (t*t) ) +
							( p3.x * (t*t*t) );

		double yVector =	( p0.y * ((1-t) * (1-t) * (1-t)) ) +
							( p1.y * ((1-t) * (1-t)) * t ) +
							( p2.y * (1-t) * (t*t) ) +
							( p3.y * (t*t*t) );

		Position vector = new Position(xVector, yVector, 0);

		return vector;
	}

	// Returns the velocity (rate of change) as a vector, which will determine the orientation of the robots' movement.
	// This is accomplished by taking the first derivative of the position function above.
	// We ignore the magnitude of the vector so that we can control the speed by other means.

	public Position getVelocityVector(Position p0, Position p1, Position p2, Position p3, double t)
	{
	    double p0Vector = -3 * Math.pow(1-t, 2);
	    double p1Vector = -6 * t * (1-t);
	    double p1Vector_= 3 * Math.pow(1-t, 2);
	    double p2Vector = -3 * Math.pow(t, 2);
	    double p2Vector_ = 6 * (1-t) * 2 * t;
	    double p3Vector = 3 * Math.pow(t, 2);

	    double xVector = (p0.x * p0Vector) +
				(p1.x * p1Vector) +
				(p1.x * p1Vector_) +
				(p2.x * p2Vector) +
				(p2.x * p2Vector_) +
				(p3.x * p3Vector);

		double yVector = (p0.x * p0Vector) +
				(p1.y * p1Vector) +
				(p1.y * p1Vector_) +
				(p2.y * p2Vector) +
				(p2.y * p2Vector_) +
				(p3.y * p3Vector);

		Position vector = new Position(xVector, yVector, 0);

		return vector;
	}

	// Returns the total length of the curve. This is an approximation based on a set number (in this case 200) of secant lines placed along the curve.

	public double getArcLength(Position p0, Position p1, Position p2, Position p3)
	{
		Position nextPos = new Position(0, 0, 0);
		Position pos = new Position(0, 0, 0);
		double deltaX, deltaY, l = 0;

		for (int i = 1; i < 2001; i++)
		{
			deltaX = nextPos.x - pos.x;
			deltaY = nextPos.y - pos.y;

			pos = nextPos;
			nextPos = getPositionVector(p0, p1, p2, p3, i/2000d);

			l += Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
		}

		return l;
	}

	// Returns an approximation of the value t which when passed to the position function would return the robot's current position.
	// This is needed by the controller to be passed to the velocity function at each iteration of the control loop.

	public double getT(double distAlongCurve, double arcLength)
	{
		return distAlongCurve / arcLength;
	}
}
