// Copyright (c) 2023 Bepu Entertainment LLC
// Copyright (c) 2023 Silverlan
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cinttypes>

namespace BEPUik
{
    /// <summary>
    /// Maps indices to permuted versions of the indices.
    /// </summary>
    class PermutationMapper
    {
    private:
		int64_t permutationIndex;
		int64_t currentOffset;
		int64_t currentPrime;

    public:
        /// <summary>
        /// Constructs a new permutation mapper.
        /// </summary>
        PermutationMapper();

        /// <summary>
        /// Gets or sets the permutation index used by the solver.  If the simulation is restarting from a given frame,
        /// setting this index to be consistent is required for deterministic results.
        /// </summary>
		void SetPermutationIndex(int64_t value);
		int64_t GetPermutationIndex();

        /// <summary>
        /// Gets a remapped index.
        /// </summary>
        /// <param name="index">Original index of an element in the set to be redirected to a shuffled position.</param>
        /// <param name="setSize">Size of the set being permuted. Must be smaller than 350000041.</param>
        /// <returns>The remapped index.</returns>
		int64_t GetMappedIndex(int64_t index, int setSize);
    };
    

}
