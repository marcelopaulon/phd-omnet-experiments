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

network DADCA_ACK_n100_u16_a8000m2_5
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
        @display("i=misc/sensor;p=153.90,7827.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=650.68,7831.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1309.42,7791.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2345.61,7968.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2900.24,7927.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4893.94,7814.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5261.60,7938.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5854.98,7891.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=974.38,7603.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1767.07,7684.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2122.66,7626.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3173.91,7676.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3863.25,7685.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4483.29,7734.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5408.33,7602.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=505.65,7422.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1470.74,7380.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2555.90,7436.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3127.45,7325.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4952.44,7367.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5694.47,7311.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1.89,7091.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1142.89,7184.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3770.24,7160.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4302.63,7152.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5318.98,7190.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5954.71,7012.93");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=919.49,6903.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2383.28,6985.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2799.42,6997.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4791.98,6820.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5414.49,6768.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1243.58,6718.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1671.78,6567.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2961.65,6519.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3444.00,6716.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4327.99,6557.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5119.19,6575.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5771.96,6582.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=383.50,6480.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2506.53,6388.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3471.37,6320.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3986.77,6445.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4683.35,6458.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=88.06,6236.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=920.62,6025.69");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1251.83,6264.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2086.20,6077.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3015.80,6033.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3916.07,6093.84");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4909.80,6142.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5358.37,6137.29");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=484.81,5951.53");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1492.50,5855.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2458.22,5938.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4439.91,5954.27");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5714.72,5933.68");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=155.10,5597.91");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=842.38,5683.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1907.90,5704.00");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2765.56,5746.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5169.81,5582.68");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1371.22,5396.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2840.65,5327.68");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3491.33,5357.37");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4137.26,5368.78");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4734.65,5459.73");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=524.36,5208.06");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1000.48,5219.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1666.69,5115.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2196.68,5093.54");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4751.19,5043.87");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5261.12,5045.74");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5658.77,5264.55");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1290.10,4815.61");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3289.31,4882.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3725.08,5028.44");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4245.21,4894.35");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=607.40,4744.18");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1876.78,4703.86");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2546.65,4685.60");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2897.77,4557.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4339.14,4546.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4955.08,4560.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5418.54,4713.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=287.81,4403.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2277.13,4335.46");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3777.00,4532.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5916.03,4393.35");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=905.12,4123.43");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1280.68,4168.83");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1637.16,4117.85");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3338.50,4188.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4147.35,4177.27");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4629.10,4096.50");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5549.46,4126.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=352.88,3913.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1956.37,3903.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2308.00,3911.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2809.82,4038.02");
    }

}

