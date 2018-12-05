using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

namespace Master
{
    using H = Master.LibQuaternionAritmetics;
    public class SetupDataStructures : ComponentSystem
    {
        public struct State
        {
            public readonly int Length;
            public ComponentArray<Transform> T;
            public ComponentArray<TravelerData> curves;
        }

        [Inject] private State state;

        protected override void OnUpdate()
        {

            //Debug.Log(state.Length);



        }
    }
}

