/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Technical University of Munich
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Toronto nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Liding Zhang, Xu Liang

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_ADAPTIVEBATCHSIZE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_ADAPTIVEBATCHSIZE_

#include "ompl/geometric/planners/informedtrees/fitstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/informedtrees/FITstar.h"
// #include "ompl/pdt/src/obstacles/include/pdt/obstacles/hyperrectangle.h"

namespace ompl
{
    namespace geometric
    {
        namespace fitstar
        {
            enum DecayMethod
            {
                ITERATION_TIMES,
                LINEAR,
                PARABOLA,
                LOG,
                BRACHISTOCHRONE,
            };

            class AdaptiveBatchSize
            {
            public:
                AdaptiveBatchSize(const DecayMethod &decay_method, ompl::base::Cost &solutionCost,
                                  const double &minPossibleCost, unsigned int &batchSize, double &S_max_initial,
                                  double &S_min_initial, const unsigned int &maxSamples,
                                  const unsigned int &minSamples);
                ~AdaptiveBatchSize() = default;

                unsigned int adjustBatchSize(DecayMethod decay_method);

            private:
                /** \brief Type of Decay method */
                unsigned int adjustBatchSizeIterationTimes();

                /** \brief Type of Decay method */
                unsigned int adjustBatchSizeLinear();

                /** \brief Type of Decay method */
                unsigned int adjustBatchSizeParabola();

                /** \brief Type of Decay method */
                unsigned int adjustBatchSizeLog();

                /** \brief Type of Decay method */
                unsigned int adjustBatchSizeBrachistochrone();

                /** \brief Type of Decay method */
                DecayMethod decay_method_;

                /** \brief Returns the start states. */
                // ompl::base::Cost _solutionCost_;
                ompl::base::Cost _solutionCost_{std::numeric_limits<double>::infinity()};

                /** \brief Returns the start states. */
                double minPossibleCost_;

                /** \brief Returns the start states. */
                unsigned int _batchSize_;

                /** \brief Returns the start states. */
                double _S_max_initial_;

                /** \brief Returns the start states. */
                double _S_min_initial_;

                /** \brief Returns the start states. */
                unsigned int _maxSamples_;

                /** \brief Returns the start states. */
                unsigned int _minSamples_;

                /** \brief Returns the start states. */
                std::size_t _iteration_;
            };

        }  // namespace fitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_ADAPTIVEBATCHSIZE_