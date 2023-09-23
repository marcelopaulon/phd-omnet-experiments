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
        @display("i=misc/sensor;p=329.09,7941.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=907.02,7993.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1380.10,7949.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1733.62,7981.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2273.98,7869.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4364.78,7968.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5030.11,7948.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=214.28,7515.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=621.99,7592.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1092.83,7629.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1535.41,7566.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2794.64,7548.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3444.55,7721.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3972.49,7554.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4642.61,7633.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5608.77,7694.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=970.23,7288.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1752.19,7277.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4878.74,7349.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5265.50,7309.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=533.52,7055.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2721.78,7071.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3406.53,7217.80");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4225.36,7097.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5870.52,7055.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=93.34,6844.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1331.87,6969.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2145.35,6944.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3018.00,6873.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4737.04,6881.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=862.97,6649.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1876.04,6629.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2566.29,6586.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3216.73,6573.22");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3786.98,6639.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4515.81,6603.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=255.12,6397.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=734.92,6321.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1255.10,6330.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2838.56,6340.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4994.05,6508.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5645.17,6410.69");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1609.13,6117.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2252.34,6175.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3245.83,6050.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3838.83,6262.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4563.58,6185.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5351.76,6104.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=504.72,5811.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=954.91,5902.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2658.06,5975.25");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3710.78,5933.10");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4266.67,5840.28");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4913.24,6007.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5650.94,5831.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=155.13,5750.52");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=844.52,5566.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1248.34,5688.06");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1604.73,5761.80");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2277.35,5616.01");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3140.34,5708.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4719.41,5653.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5129.37,5548.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2564.78,5375.80");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3102.09,5355.50");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3517.49,5372.96");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5486.61,5377.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5820.82,5519.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=124.01,5195.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=589.65,5272.56");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1797.98,5238.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4396.75,5146.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5143.83,5136.03");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=460.21,4828.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=870.73,4962.49");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1229.69,5005.75");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1691.72,4851.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3914.33,4840.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5463.91,4979.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5928.17,4996.58");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=6.42,4561.68");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3485.57,4737.31");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4752.81,4727.65");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=602.35,4433.63");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=952.89,4504.53");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1433.01,4334.12");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3189.87,4415.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3786.65,4492.55");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4136.67,4524.82");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5660.53,4530.78");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2083.10,4246.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3680.58,4133.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4099.58,4098.76");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4494.70,4251.01");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4975.34,4274.45");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=209.08,3961.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1097.89,4000.17");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1526.33,3953.81");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2045.35,3884.44");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2769.52,3894.34");
    }

}

