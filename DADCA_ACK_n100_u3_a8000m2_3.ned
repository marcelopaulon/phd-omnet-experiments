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

network DADCA_ACK_n100_u3_a8000m2_3
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
        @display("i=misc/sensor;p=1881.04,7823.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2435.71,7807.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4059.03,7929.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4696.73,7770.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5099.01,7773.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5755.05,7785.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=422.86,7721.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1376.31,7544.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2137.26,7574.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5489.01,7527.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=869.17,7409.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3611.21,7496.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4497.36,7457.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=568.10,7160.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1407.98,7103.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1969.32,7214.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2410.67,7038.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2768.74,7036.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3363.30,7222.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4023.34,7060.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5043.83,7248.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=243.23,6780.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1051.60,6912.29");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3161.96,6836.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4524.02,6815.16");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5888.43,7006.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1589.28,6663.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2143.01,6731.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5454.19,6721.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=111.95,6390.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=738.03,6505.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1134.84,6415.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2628.87,6311.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2990.02,6380.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3364.64,6372.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3889.60,6504.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4304.17,6386.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5192.60,6286.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5841.02,6383.33");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=683.44,6159.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1466.04,6098.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1941.35,6234.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4789.66,6080.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=175.89,5868.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2473.05,5994.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3092.36,5891.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3855.95,5814.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4434.83,6009.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5474.92,6009.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=477.49,5547.43");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1047.25,5714.80");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1697.94,5698.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2322.33,5578.09");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2821.71,5664.32");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4953.15,5722.62");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5359.65,5676.47");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5870.67,5747.80");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=151.21,5323.99");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=785.94,5339.95");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1305.63,5399.14");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2898.24,5283.93");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3385.70,5379.24");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4335.73,5483.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5692.33,5365.27");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1210.09,5050.00");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1985.23,5246.66");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4670.82,5130.74");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5256.32,5044.67");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=27.36,4820.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=411.32,4896.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=901.84,4803.64");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1598.42,4820.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2599.81,4982.25");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3350.27,4828.25");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3785.26,4975.17");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4255.58,5020.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=601.50,4569.23");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2029.49,4604.01");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2483.85,4571.95");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2969.02,4697.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3987.34,4569.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4819.88,4651.51");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5174.40,4679.50");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5882.86,4756.61");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1136.70,4384.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1541.05,4452.59");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3595.70,4490.44");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4379.78,4291.22");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=309.53,4215.82");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=723.33,4064.34");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1617.10,4090.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2654.55,4193.83");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3578.90,4041.17");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5150.99,4102.41");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5782.26,4279.15");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1264.43,3856.94");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2227.84,4029.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4051.37,3902.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4578.06,3913.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=114.11,3691.63");
    }

}

