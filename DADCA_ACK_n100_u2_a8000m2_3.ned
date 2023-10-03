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

network DADCA_ACK_n100_u2_a8000m2_3
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
        @display("i=misc/sensor;p=228.63,7846.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1200.53,7862.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4712.80,7763.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=689.12,7510.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1735.82,7614.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2318.22,7580.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3093.61,7733.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3535.84,7554.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3872.21,7697.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=30.34,7495.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1134.94,7419.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1523.46,7267.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4098.00,7262.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5106.19,7444.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=179.38,7071.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=840.72,7095.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2848.82,7185.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3392.79,7222.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4438.69,7136.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4967.65,7113.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1399.78,6899.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1842.88,6939.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2559.66,6955.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3672.40,6904.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5415.55,6912.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=237.62,6609.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=713.64,6590.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1087.30,6732.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2606.08,6552.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3068.33,6601.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4166.60,6740.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5088.80,6533.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1131.40,6352.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4122.46,6381.85");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5560.13,6274.20");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=462.71,6225.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1494.38,6225.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2696.35,6187.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3631.47,6222.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3947.97,6025.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4619.53,6039.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=942.52,5996.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1339.65,5774.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2117.28,5915.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2671.45,5782.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3279.05,5916.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5063.59,5817.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=213.40,5689.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=611.45,5693.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1818.70,5698.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4070.96,5566.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4606.28,5586.57");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5391.99,5648.85");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=897.93,5417.92");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2314.24,5340.99");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2733.07,5389.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3256.85,5433.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3710.11,5391.42");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=315.40,5128.75");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1026.68,5046.24");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1407.98,5251.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1932.14,5263.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3091.60,5046.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4262.34,5092.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4942.73,5063.85");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5349.32,5121.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2423.47,4827.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3513.59,4933.39");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5794.78,4909.98");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=318.21,4742.23");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=799.10,4661.40");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1202.42,4674.68");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1566.73,4740.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2043.20,4702.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2726.22,4615.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3950.04,4591.28");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4451.04,4557.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5096.34,4561.39");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=579.12,4377.65");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=932.96,4310.34");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1440.20,4407.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3403.25,4421.41");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5575.23,4339.92");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=311.04,4120.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1809.86,4121.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2419.69,4236.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2833.27,4073.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4011.51,4113.21");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5136.58,4108.94");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5891.39,4076.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=644.82,3977.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1455.55,4001.43");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3504.46,4007.48");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4677.44,3950.69");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5381.43,3818.39");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1012.46,3746.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1634.92,3595.05");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2178.86,3621.38");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2623.72,3721.71");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3775.73,3660.73");
    }

}

