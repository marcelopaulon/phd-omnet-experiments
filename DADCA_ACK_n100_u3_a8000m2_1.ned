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

network DADCA_ACK_n100_u3_a8000m2_1
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
        @display("i=misc/sensor;p=163.85,7922.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1076.61,7992.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2136.07,7982.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2920.19,7921.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3580.52,7761.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4082.41,7787.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5484.83,7841.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=693.64,7741.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1535.18,7671.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2181.25,7599.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2531.11,7568.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4546.14,7676.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=27.46,7276.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=477.28,7300.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=906.51,7462.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1299.02,7348.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1933.75,7264.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3197.71,7403.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3858.20,7365.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4968.18,7458.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5590.69,7354.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2632.47,7097.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3530.36,7103.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4422.92,7257.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5069.02,7037.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5944.36,7232.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=909.27,6919.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3083.52,6975.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3963.38,6786.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4502.45,6812.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5608.58,6914.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=16.78,6565.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=386.86,6757.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=904.58,6538.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1323.74,6759.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5263.06,6647.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=524.26,6298.57");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1756.77,6432.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2873.37,6382.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3338.97,6281.57");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3849.58,6415.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4262.02,6378.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4839.89,6467.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5729.07,6450.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=912.22,6182.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2238.46,6221.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5133.47,6113.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5496.92,6168.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=96.86,5930.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1490.54,5952.35");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2706.51,6014.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3084.37,5997.84");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3717.53,6008.76");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4647.15,5817.30");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5936.00,5827.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=625.86,5606.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1063.27,5606.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2194.23,5750.09");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3186.43,5656.00");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3780.96,5528.40");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4283.96,5745.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5104.50,5749.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=33.84,5341.18");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1702.18,5340.75");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2223.87,5394.26");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2588.83,5423.57");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4870.80,5398.24");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5357.08,5463.28");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3715.07,5106.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4330.40,5152.47");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5707.60,5249.53");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=340.58,4813.65");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=785.73,5020.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1220.65,4969.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2515.76,4839.71");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3233.02,4960.77");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4521.94,4793.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5169.50,4934.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5564.18,4795.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=952.22,4683.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1548.09,4710.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4107.76,4630.67");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=26.62,4362.82");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=375.22,4425.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1382.91,4368.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1962.18,4514.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2246.85,4296.05");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2965.12,4370.82");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3447.25,4501.97");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3851.36,4312.17");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4353.01,4302.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4939.61,4342.75");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5971.18,4476.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5283.01,4247.88");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5718.39,4207.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=593.26,4024.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1037.28,4038.53");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1497.80,4030.37");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1962.42,3921.98");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2595.09,4013.82");
    }

}

