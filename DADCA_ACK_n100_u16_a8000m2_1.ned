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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=523.65,7823.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1089.88,7934.77");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1539.12,7800.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2897.35,7892.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3600.59,7867.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4444.29,7834.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5706.13,7948.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=234.72,7609.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2438.68,7559.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4008.49,7524.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5000.87,7663.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5980.15,7505.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=891.66,7389.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1328.57,7302.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2143.15,7260.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4510.79,7384.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4876.74,7298.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5530.61,7455.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=208.47,7111.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1865.77,7014.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3085.07,7253.77");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3584.72,7105.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4088.62,7101.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5522.43,7035.23");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=603.77,6876.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=969.51,6829.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2586.03,6992.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4906.07,6859.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5982.52,6932.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=53.47,6620.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1616.30,6604.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2137.30,6528.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2680.22,6646.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3331.48,6632.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4171.42,6726.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4672.71,6544.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5414.41,6604.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5777.77,6636.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=738.93,6397.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2471.02,6289.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3824.85,6508.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4151.27,6365.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5148.42,6286.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=163.15,6218.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1190.72,6243.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2121.47,6090.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3507.05,6026.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3900.25,6049.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5422.36,6025.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1013.89,5835.02");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1489.39,5919.43");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4548.03,5864.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5970.44,5983.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=548.19,5722.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2309.11,5686.35");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2785.48,5611.35");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4212.55,5715.11");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5473.94,5602.60");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5978.15,5529.49");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=304.59,5325.44");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=959.45,5360.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1951.40,5371.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3451.74,5431.14");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3871.06,5446.26");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4430.31,5358.14");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4929.73,5503.49");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=556.38,5057.78");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1615.53,5119.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2361.36,5260.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3129.17,5270.24");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4196.57,5068.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5072.81,5107.45");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5631.02,5041.67");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1092.82,4861.44");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2096.42,4934.08");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2782.08,4920.61");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3156.12,4831.98");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3600.01,4863.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4484.31,4793.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=110.19,4697.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=617.24,4708.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2697.09,4567.77");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4109.66,4708.36");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4833.48,4705.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5830.41,4609.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=353.32,4405.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=791.14,4333.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1444.46,4485.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2315.42,4481.71");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3056.69,4432.39");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3810.92,4405.24");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4543.39,4438.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1383.23,4042.81");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1716.83,4223.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2227.40,4112.39");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2969.48,4090.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3515.87,4067.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4961.73,4160.30");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5323.31,4285.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5866.74,4101.03");
    }

}

