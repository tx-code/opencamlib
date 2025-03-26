/*  $Id$
 *
 *  Copyright (c) 2010 Anders Wallin (anders.e.e.wallin "at" gmail.com).
 *
 *  This file is part of OpenCAMlib
 *  (see https://github.com/aewallin/opencamlib).
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CCPOINT_H
#define CCPOINT_H

#include <iostream>
#include <string>

#include "point.hpp"

namespace ocl {
    /// type of cc-point
    enum CCType {
        NONE,
        VERTEX,
        VERTEX_CYL,
        EDGE,
        EDGE_HORIZ,
        EDGE_SHAFT,
        EDGE_HORIZ_CYL,
        EDGE_HORIZ_TOR,
        EDGE_BALL,
        EDGE_POS,
        EDGE_NEG,
        EDGE_CYL,
        EDGE_CONE,
        EDGE_CONE_BASE,
        FACET,
        FACET_TIP,
        FACET_CYL,
        CCTYPE_ERROR
    };

    inline std::string CCType2String(CCType type) {
        switch (type) {
            case NONE:
                return "NONE";
            case VERTEX:
                return "VERTEX";
            case VERTEX_CYL:
                return "VERTEX_CYL";
            case EDGE:
                return "EDGE";
            case EDGE_HORIZ:
                return "EDGE_HORIZ";
            case EDGE_SHAFT:
                return "EDGE_SHAFT";
            case EDGE_HORIZ_CYL:
                return "EDGE_HORIZ_CYL";
            case EDGE_HORIZ_TOR:
                return "EDGE_HORIZ_TOR";
            case EDGE_BALL:
                return "EDGE_BALL";
            case EDGE_POS:
                return "EDGE_POS";
            case EDGE_NEG:
                return "EDGE_NEG";
            case EDGE_CYL:
                return "EDGE_CYL";
            case EDGE_CONE:
                return "EDGE_CONE";
            case EDGE_CONE_BASE:
                return "EDGE_CONE_BASE";
            case FACET:
                return "FACET";
            case FACET_TIP:
                return "FACET_TIP";
            case FACET_CYL:
                return "FACET_CYL";
            case CCTYPE_ERROR:
            default:
                return "ERROR";
        }
    }

    ///
    /// \brief Cutter-Contact (CC) point. A Point with a CCType.
    ///
    /// Cutter-Contact (CC) Point.
    /// A Point which also contains the type of cutter-contact.
    class OCL_API CCPoint : public Point {
    public:
        /// create a CCPoint at (0,0,0)
        CCPoint();

        /// create CCPoint at (x,y,z)
        CCPoint(double x, double y, double z);

        /// create CCPoint at (x,y,z) with type t
        CCPoint(double x, double y, double z, CCType t);

        /// create CCPoint at p with type t
        CCPoint(const Point& p, CCType t);

        /// create a CCPoint at Point p
        CCPoint(const Point& p);

        virtual ~CCPoint() {
        };

        /// specifies the type of the Cutter Contact point.
        CCType type;

        /// assign coordinates of Point to this CCPoint. sets type=NONE
        CCPoint& operator=(const Point& p);

        /// string repr
        std::string str() const;

        /// string repr
        friend std::ostream& operator<<(std::ostream& stream, const CCPoint& p);
    };
} // namespace ocl
#endif
// end file ccpoint.h
