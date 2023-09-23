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

network DADCA_ACK_n100_u4_a8000m2_5
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
        @display("i=misc/sensor;p=1162.57,7955.14");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1520.56,7928.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2802.42,7782.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4779.90,7995.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5678.43,7949.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=707.12,7748.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1021.70,7506.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1876.34,7534.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3317.62,7550.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3910.32,7583.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4530.66,7631.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5057.12,7652.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=581.72,7389.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1447.54,7471.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2710.40,7326.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5610.16,7320.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=149.20,7125.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=721.42,7024.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1657.06,7058.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2164.46,7049.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3265.12,7099.29");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3983.52,7098.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4617.50,7090.38");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1118.55,6829.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4249.68,6777.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4829.72,6779.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=223.64,6529.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=893.73,6543.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1617.59,6651.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2525.82,6686.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2973.34,6724.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3587.89,6717.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1230.75,6334.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2223.06,6486.42");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4049.22,6323.92");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4491.73,6379.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5293.21,6273.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=105.60,6089.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1639.58,6151.39");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2915.99,6107.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3457.65,6123.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4913.02,6232.52");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1004.84,5885.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2027.90,5994.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2495.86,5943.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5386.79,5833.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=440.69,5643.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=791.44,5580.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1619.34,5736.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1963.33,5627.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3182.15,5571.04");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3903.36,5639.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4530.47,5693.25");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5023.51,5713.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5663.05,5541.71");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=22.27,5308.39");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1668.16,5326.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2289.24,5449.20");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2768.87,5508.50");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5299.32,5287.93");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=376.35,5235.38");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1116.77,5226.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2047.00,5071.01");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2707.68,5039.41");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3724.69,5176.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4081.33,5264.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4707.58,5269.59");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=339.34,4810.24");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4644.08,4889.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5336.93,4896.11");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=950.23,4683.29");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1571.75,4636.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3536.81,4776.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4136.72,4695.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5775.66,4552.40");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2018.70,4500.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2494.62,4530.67");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3068.71,4396.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3942.91,4378.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5174.57,4396.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=813.90,4286.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1225.81,4235.77");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1691.94,4244.74");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2332.00,4152.16");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3594.85,4054.60");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4592.92,4232.62");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4991.22,4055.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=322.68,3808.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1772.95,3868.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2782.95,3913.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3300.46,3856.40");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3984.80,4014.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4392.20,3899.03");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5563.84,4027.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=865.81,3723.32");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1320.45,3657.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2286.36,3668.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4825.83,3697.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5917.95,3772.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=497.10,3450.94");
    }

}

