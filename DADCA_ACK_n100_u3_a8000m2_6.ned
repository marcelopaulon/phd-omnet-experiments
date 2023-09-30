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

network DADCA_ACK_n100_u3_a8000m2_6
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
        @display("i=misc/sensor;p=328.27,7947.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1008.37,7858.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2199.27,7879.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2972.88,7782.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3626.01,7821.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4153.43,7914.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4991.77,7851.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5898.79,7970.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=777.96,7552.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1798.31,7687.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4376.92,7609.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5247.11,7593.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5644.90,7652.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=239.06,7294.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2575.62,7479.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3520.20,7361.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4056.35,7349.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4868.56,7487.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1366.92,7231.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2141.69,7200.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4608.70,7181.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5248.44,7088.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5759.90,7144.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=475.01,6959.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1008.05,6852.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1752.62,6836.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2444.58,6886.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2862.31,6898.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3269.47,6930.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3686.71,6983.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4208.15,6832.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4829.72,6896.06");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=17.18,6609.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=510.76,6579.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1257.20,6525.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2224.53,6608.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3707.44,6611.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4539.51,6593.72");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5516.54,6751.75");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5929.23,6597.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2493.81,6374.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3337.73,6482.58");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4035.61,6467.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4973.91,6467.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1281.76,6022.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2252.06,6033.40");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4283.12,6171.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5371.84,6150.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5923.41,6155.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=246.09,5835.45");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=780.27,5985.54");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1642.57,5965.61");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1962.88,5802.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2555.20,5773.62");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3238.58,5994.04");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3921.04,5911.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=542.99,5577.28");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1055.53,5732.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2884.35,5564.08");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3389.83,5617.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5238.30,5585.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5635.54,5668.41");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1448.20,5423.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1853.91,5325.93");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2277.96,5461.41");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4368.47,5473.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5005.57,5289.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5961.47,5387.03");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=305.57,5215.68");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1036.87,5158.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2304.44,5096.60");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3993.28,5190.29");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4624.64,5079.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5627.49,5189.58");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=567.24,4962.84");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1632.56,4849.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2820.65,4995.60");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3200.73,4968.05");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3683.91,4999.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4297.93,4857.79");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=185.23,4562.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=904.40,4700.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2250.05,4663.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2734.01,4630.04");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3364.87,4540.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3746.52,4590.87");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4955.18,4765.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5324.13,4675.75");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5934.30,4733.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=804.02,4308.14");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1340.73,4291.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1937.14,4489.97");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4385.13,4442.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4852.64,4426.59");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=370.80,4259.81");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3019.71,4096.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3598.30,4253.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4597.99,4161.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5052.30,4068.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5469.73,4097.72");
    }

}

