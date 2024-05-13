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
        @display("i=misc/sensor;p=87.67,7884.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1146.02,7964.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1542.30,7812.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3078.90,7816.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3691.20,7812.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5708.45,7970.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=707.20,7677.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1064.03,7567.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2710.73,7712.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4128.10,7602.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4755.31,7678.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5113.16,7685.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1544.87,7381.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2114.81,7423.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2715.54,7273.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3299.40,7335.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5525.44,7405.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=24.28,7195.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=581.02,7244.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1073.05,7190.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3696.40,7205.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4094.65,7067.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4639.90,7138.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5151.25,7096.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5918.56,7237.50");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1459.37,6990.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2207.43,6962.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3145.29,6846.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4455.27,6798.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5489.10,6798.30");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=580.00,6702.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1171.90,6698.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1526.66,6530.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2661.20,6650.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3752.27,6651.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4990.14,6757.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=371.90,6333.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=852.81,6372.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1892.88,6410.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2351.28,6440.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4342.07,6309.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5470.91,6419.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5872.79,6435.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1474.20,6128.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3122.79,6212.70");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3551.06,6071.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3993.34,6024.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4728.13,6172.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5171.21,6183.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5934.35,6056.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=922.97,5870.02");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1988.57,5996.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2806.01,5994.85");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4870.20,5787.26");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5233.51,5819.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=478.36,5651.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1663.48,5557.60");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2069.60,5556.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2868.03,5553.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3161.65,5746.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3495.75,5550.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=980.56,5414.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1355.00,5340.10");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2420.73,5381.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4278.27,5503.76");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4879.55,5426.45");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5771.18,5490.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1724.69,5103.53");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3392.75,5189.94");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3905.73,5107.13");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=78.40,4999.40");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=656.03,5014.21");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1235.54,4978.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1554.93,4785.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2379.92,5027.94");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2931.83,4952.60");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4840.32,4822.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5410.01,4991.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5821.49,5010.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2465.22,4589.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3456.52,4752.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4230.00,4590.68");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5961.35,4584.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=440.70,4435.45");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=945.66,4400.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1925.85,4520.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2950.77,4441.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3751.28,4528.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4565.36,4490.04");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5031.21,4410.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5519.38,4399.44");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2497.88,4160.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3374.48,4144.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4255.76,4232.43");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5894.35,4201.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=208.70,4026.73");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=903.70,3858.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1628.06,3932.07");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2025.10,3817.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2940.87,3990.42");
    }

}

