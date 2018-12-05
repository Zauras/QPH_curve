using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

namespace Master
{
    public static class Database
    {
        public static Dictionary<int, Entity> worldPaths = new Dictionary<int, Entity>();
        //public static List<List<int>> worldCurvesIDs = new List<List<int>>();
        public static Dictionary<int, GameObject> ID_GOmap = new Dictionary<int, GameObject>();

        //public static Dictionary<int, DualQuaternion[]> PathsDisplacementQuaternions 
         //   = new Dictionary<int, DualQuaternion[]>(); // <pathID, [Wt,Ft]>


       // public static Dictionary<int, DisplacementData[]> ObjectMovementPath = new Dictionary<int, DisplacementData[]>(); // <pathID, trajectory{pos & rot>
        public static Dictionary<int, int> MovementProgress = new Dictionary<int, int>(); // <pathID, counterInTrajectoryProgress>

        public static Dictionary<int, Entity> Travelers = new Dictionary<int, Entity>(); // pathID, travalerEntity
    }


}
