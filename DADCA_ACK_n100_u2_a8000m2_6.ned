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

network DADCA_ACK_n100_u2_a8000m2_6
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
        @display("i=misc/sensor;p=692.77,7834.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1828.75,7974.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2868.78,7863.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3326.44,7789.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=228.78,7669.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1033.01,7731.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1559.93,7566.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2190.79,7729.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3870.58,7603.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4567.12,7590.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5420.53,7636.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=349.09,7270.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=743.59,7316.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1225.12,7345.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1827.79,7269.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2732.42,7465.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3095.27,7495.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5332.73,7263.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2482.76,7133.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3101.09,7016.59");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3387.56,7232.75");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3875.55,7201.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4421.51,7186.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4946.58,7147.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=176.16,6905.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=832.20,6937.26");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3903.49,6772.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4995.84,6786.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5382.23,6789.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=144.75,6554.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=509.00,6559.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1191.53,6665.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1563.51,6529.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1965.31,6705.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2530.58,6641.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2941.66,6548.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4511.56,6604.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5707.83,6566.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=851.80,6444.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3608.81,6461.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5163.88,6359.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1273.29,6142.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1731.74,6161.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2336.07,6159.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2765.50,6116.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4220.23,6195.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4717.80,6223.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5380.29,6031.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5811.23,6122.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=704.52,5861.33");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1875.81,5814.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3175.31,5985.43");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3637.24,5955.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4396.78,5883.52");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1439.92,5609.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2510.04,5670.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3053.84,5615.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4052.19,5724.37");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5239.40,5569.33");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5568.36,5731.22");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5899.68,5541.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=110.75,5492.35");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=758.35,5361.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1113.48,5366.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1900.07,5287.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3857.46,5421.31");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4831.54,5511.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2749.73,5258.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3214.05,5261.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3556.32,5183.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4325.59,5217.19");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5371.70,5062.43");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=356.58,4957.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1351.03,4944.83");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2356.05,4898.70");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2924.65,4950.71");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3824.92,4797.68");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4832.44,4951.37");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=112.81,4669.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=857.44,4711.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1769.21,4777.10");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4476.44,4718.60");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5112.56,4608.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5856.42,4614.73");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1165.38,4523.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1990.15,4456.16");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2670.88,4357.24");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3159.96,4308.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3466.32,4514.68");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4079.60,4292.75");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=378.59,4276.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=711.62,4086.83");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1179.35,4147.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4910.32,4244.30");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5604.19,4183.67");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1475.63,3911.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2298.87,3878.68");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2941.41,3990.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3631.50,3928.16");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4495.23,3908.19");
    }

}

