package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u2_a8000m2_2
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=877.12,7972.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1617.02,7937.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2200.74,7978.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2809.22,7953.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3310.35,7884.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4503.79,7964.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5192.43,7871.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5636.30,7940.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=478.91,7681.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=948.36,7587.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1417.48,7588.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2216.74,7628.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3970.18,7703.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1234.56,7280.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1883.81,7390.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2658.14,7357.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3522.27,7327.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4294.81,7417.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4916.33,7485.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5283.81,7275.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5628.68,7480.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=48.20,7151.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1666.83,7109.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2055.22,7068.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3185.22,7120.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3880.23,7223.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5889.70,7074.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=582.02,6885.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1206.01,6885.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2701.00,6849.30");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3088.04,6772.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4277.77,6928.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4816.07,6834.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5507.33,6913.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1612.43,6623.88");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2101.59,6578.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3503.50,6611.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3975.65,6727.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5185.18,6678.88");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=913.46,6391.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1281.65,6487.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2596.49,6357.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3003.90,6393.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4396.23,6501.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4915.25,6412.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5522.28,6511.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=65.88,6240.42");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=424.84,6244.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1058.49,6069.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1697.78,6170.89");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3844.44,6151.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4271.97,6136.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=100.69,5837.47");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1394.77,5921.16");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2396.72,6006.69");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3438.57,5918.28");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4581.55,5865.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4944.26,5908.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5347.54,5986.09");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=464.24,5715.79");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2014.39,5739.02");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2919.12,5551.03");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3884.28,5763.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5525.57,5632.52");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5900.99,5708.52");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=238.80,5325.71");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=993.07,5302.22");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1557.80,5433.28");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2071.89,5353.24");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3198.61,5295.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3863.02,5325.81");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4315.10,5332.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5026.26,5516.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5911.06,5334.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1613.82,5084.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2559.83,5059.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5104.71,5141.44");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=826.01,4793.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1321.15,4822.43");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3028.18,4796.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3691.37,4825.14");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4034.29,4984.02");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=57.91,4726.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2043.68,4583.55");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2634.48,4559.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3375.22,4616.76");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4508.34,4671.38");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5463.73,4594.47");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=989.38,4421.17");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4659.42,4353.86");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5052.67,4447.39");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=419.12,4243.44");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1658.83,4117.05");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2029.91,4227.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2599.67,4160.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3243.58,4209.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3781.91,4159.64");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4133.17,4194.47");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5416.69,4118.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5822.51,4151.27");
    }

}

