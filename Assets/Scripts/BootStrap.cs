using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine.SceneManagement;

namespace Master
{
    public sealed class BootStrap
    {
        public static MeshInstanceRenderer VectorArrowLook;
        public static MasterSettings Settings;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        public static void Initialize()
        {
           // Debug.Log("Initialize()");
            // Init Archetypes:
            //Archetypes.SetArchetype();
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        public static void InitializeAfterSceneLoad()
        {   
           // Debug.Log("1. BootStrap: InitializeAfterSceneLoad()");

            var settingsGO = GameObject.Find("Settings");
            if (settingsGO == null)
            {
                SceneManager.sceneLoaded += OnSceneLoaded;
                return;
            }
            InitializeWithScene();
        }

        public static void InitializeWithScene()
        {
            Debug.Log("2. BootStrap: InitializeWithScene()");
            // Don't touch - just init Settings GameObject
            EntityManager em = World.Active.GetOrCreateManager<EntityManager>();


            var settingsGO = GameObject.Find("Settings");
            if (settingsGO == null)
            {
                SceneManager.sceneLoaded += OnSceneLoaded;
                return;
            }
            Settings = settingsGO?.GetComponent<MasterSettings>();
            if (!Settings) return;

            // Setup path data
            foreach (GameObject path in GameObject.FindGameObjectsWithTag("Path"))
            {
                //LineRendererSystem.AddLineToPathGO(path);
            }



            //CalculateTimeListSystem.CreateCurve(); // calc common info
            //SetupEntitiesSystem.EntitiseGOHierarchy(); // gather GO & make Entities Hierarch

            // Start daemon Systems:
        }


        private static void OnSceneLoaded(Scene scene, LoadSceneMode loadSceneMode)
        {
            InitializeWithScene();
        }

        private static MeshInstanceRenderer GetLookFromPrototype(string protoName)
        {
            var proto = GameObject.Find(protoName);
            var result = proto.GetComponent<MeshInstanceRendererComponent>().Value;
            Object.Destroy(proto);
            return result;

            // Now Gather Meshs(Looks) & save into MeshRenderer Components:
            //DroneLook = GetLookFromPrototype("DroneRenderPrototype");
        }


        /*

         // Init all Game Objects & convert to Entities with wrapper of Monos
         public static EntityArchetype droneArchetype;
         public static EntityArchetype vecArrowArchetype;
         public static EntityArchetype pointIndicatorArchetype;

         // Is it meshes? - Game Objects from GO_Prototypes in Scene
         public static MeshInstanceRenderer droneLook;
         public static MeshInstanceRenderer vecArrowLook;
         public static MeshInstanceRenderer pointIndicatorLook;


         [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
         public static void Initialize() {
             EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();
             // Creation of Entities:
             droneArchetype = entityManager.CreateArchetype(typeof(Position), typeof(TransformMatrix));
             vecArrowArchetype = entityManager.CreateArchetype(typeof(Position), typeof(TransformMatrix));
             pointIndicatorArchetype = entityManager.CreateArchetype(typeof(Position), typeof(TransformMatrix));
         }

         [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
         public static void IniatializeWithScene() {
             // Copy mesh & texture stuff from Prototypes in scene
             droneLook = GetLookFromPrototype("Drone_Prototype");
             vecArrowLook = GetLookFromPrototype("VecArrow_Prototype");
             pointIndicatorLook = GetLookFromPrototype("PointIndicator_Prototype");

             // Calculate Path (Displacement array of mesh vex)


             // daemon system? wich would use onUpdate?
             World.Active.GetOrCreateManager<HUDSystem>().Setup();
         }




         // HUD methods?
         public static void MoveByStep()
         {
            // TreeSpawnSystem.StartGenetateMap();s
            // DisplacementSystem.MoveByStep;
         }

         public static void MoveConstant()
         {
           //  TreeSpawnSystem.StartClearMap();
           // DisplacementSystem.MoveConstant();
         }

         */

    }
 
}
