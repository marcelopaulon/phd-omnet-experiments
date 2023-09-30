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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=72.94,7875.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=469.07,7948.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1101.12,7917.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1770.56,7993.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2892.46,7801.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=745.34,7662.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1383.41,7647.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2070.80,7524.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4657.93,7714.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5087.40,7661.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=206.36,7274.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1739.21,7371.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2530.11,7431.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2989.09,7408.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3685.61,7384.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4200.96,7488.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1322.87,7242.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3168.10,7025.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3981.59,7099.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4330.49,7150.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5042.25,7201.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5618.89,7251.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=659.40,6983.52");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1186.68,6825.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1757.15,6991.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2218.96,6990.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4535.77,6800.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5519.65,6765.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=304.56,6672.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=878.84,6621.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2056.62,6583.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2579.90,6698.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3378.92,6597.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3937.51,6565.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5045.60,6579.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=45.54,6330.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=637.09,6346.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1384.36,6420.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3039.73,6427.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4701.35,6451.77");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1116.81,6126.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2303.19,6127.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2652.90,6222.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3330.12,6062.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3778.43,6131.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4476.73,6178.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4832.91,6119.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5251.19,6049.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=35.42,5912.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=437.30,6015.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1447.80,5894.75");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1791.97,5969.66");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2743.68,5877.94");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4256.84,5792.28");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5001.29,5786.66");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5646.25,5900.71");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=782.01,5589.40");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1128.79,5639.63");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2047.66,5528.93");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3094.46,5762.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5972.04,5573.63");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=309.29,5468.81");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1499.82,5305.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2502.33,5461.65");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3296.91,5331.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3665.10,5470.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5112.42,5412.13");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5464.88,5341.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=131.25,5105.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=760.54,5143.27");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1144.09,5097.33");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2047.93,5101.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2578.93,5109.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3029.30,5097.01");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4088.37,5066.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4573.42,5219.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5810.33,5183.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=429.66,4873.01");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1497.78,4784.26");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3588.76,4834.45");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5099.19,4837.40");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2614.42,4624.36");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3148.19,4691.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4714.71,4760.47");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5336.00,4551.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5749.39,4639.12");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=269.73,4504.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=972.41,4436.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1333.14,4436.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1904.50,4440.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2259.93,4424.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3571.34,4419.10");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4184.77,4381.51");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3099.77,4250.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3442.52,4042.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4586.82,4239.05");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5192.98,4111.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=364.66,3991.73");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=973.05,3962.81");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1663.25,3842.81");
    }

}

