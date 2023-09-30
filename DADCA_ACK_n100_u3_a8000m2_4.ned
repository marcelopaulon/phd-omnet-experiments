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

network DADCA_ACK_n100_u3_a8000m2_4
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
        @display("i=misc/sensor;p=1279.63,7959.31");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1935.37,7991.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3901.19,7977.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4686.63,7756.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5322.91,7911.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5726.40,7786.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=638.31,7587.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=967.62,7746.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2542.68,7673.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3439.69,7613.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4125.13,7663.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=36.69,7296.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1216.92,7277.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1648.39,7264.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4943.75,7458.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5610.16,7393.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=307.00,7015.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=818.05,7070.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2469.13,7097.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3141.07,7234.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4614.46,7230.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1632.27,6828.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1982.08,6931.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3433.47,6969.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3820.39,6919.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4192.06,6961.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5964.38,6864.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=308.36,6546.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=707.93,6523.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1215.24,6587.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2836.57,6608.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3661.83,6558.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4902.51,6682.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5450.78,6615.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1557.48,6382.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2204.94,6417.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4486.72,6438.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5617.90,6307.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=274.00,6042.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=826.53,6167.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1173.36,6074.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2443.33,6139.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2967.18,6135.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3528.30,6164.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4162.20,6085.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5040.14,6111.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1709.73,5806.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2064.39,5827.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3834.28,5922.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4525.36,5826.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5932.08,5824.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1079.12,5714.67");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2581.39,5750.08");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3016.10,5737.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5594.56,5709.53");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=547.15,5516.03");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2161.81,5378.36");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2520.65,5352.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4002.38,5405.69");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4412.54,5444.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4755.15,5359.72");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=9.89,5253.20");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=703.35,5184.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1782.25,5151.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3611.23,5258.57");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4083.90,5064.11");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5198.70,5230.23");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5889.13,5180.65");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=95.76,4808.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=486.60,4800.63");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1060.09,4941.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2119.93,4935.14");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2591.19,4895.91");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3386.81,4934.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4491.66,4956.03");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4928.62,4962.63");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5406.38,4927.13");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1493.25,4761.69");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3066.93,4621.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3741.32,4688.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4155.60,4699.29");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4652.21,4578.55");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5104.33,4658.43");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=182.54,4441.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=841.64,4300.22");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1189.03,4353.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1722.82,4432.77");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2129.13,4325.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2638.62,4356.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5709.82,4491.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=393.54,4067.91");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3335.99,4282.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3915.28,4232.27");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5345.31,4242.92");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=41.96,3854.65");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=843.88,3865.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1349.28,3950.92");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2420.88,3881.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2930.50,3884.44");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3711.51,3833.15");
    }

}

