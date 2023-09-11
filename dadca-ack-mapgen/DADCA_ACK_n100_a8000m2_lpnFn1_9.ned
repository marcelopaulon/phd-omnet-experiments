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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=52.38,12.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=15.83,67.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=44.64,147.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=130.02,36.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=101.74,174.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=77.64,269.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=32.50,316.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=95.00,377.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=172.78,334.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=143.14,259.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=215.21,254.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=307.23,257.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=370.37,247.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=433.17,304.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=458.15,243.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=409.46,182.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=553.83,261.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=569.10,322.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=465.01,377.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=502.15,323.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=379.92,345.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=245.32,177.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=176.24,182.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=244.80,318.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=304.25,396.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=364.76,405.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=499.46,456.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=432.05,496.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=397.47,549.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=349.46,148.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=521.26,147.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=351.85,54.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=670.78,389.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=719.47,446.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=722.48,516.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=680.79,562.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=616.55,570.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=632.34,476.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=559.65,485.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=764.25,597.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=811.60,545.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=792.64,450.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=824.38,327.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=571.28,406.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=766.98,363.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=454.77,622.22");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=536.49,607.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=685.73,679.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=564.66,668.73");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=643.40,787.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=576.62,784.27");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=569.93,845.25");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=560.95,927.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=496.75,955.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=486.35,1047.41");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=431.40,1094.61");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=616.23,1056.06");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=628.30,1151.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=554.63,1218.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=605.20,1263.67");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=735.17,1181.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=685.01,1119.74");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=757.02,1072.44");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=831.15,1100.87");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=698.34,988.05");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=637.81,983.01");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=615.17,893.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=707.76,899.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=800.10,869.63");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=782.28,797.58");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=739.62,741.14");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=507.52,743.77");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=421.26,783.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=377.49,736.23");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=366.99,626.13");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=389.45,860.65");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=544.86,1104.35");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=690.25,1222.94");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=721.21,1293.73");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=777.13,1248.12");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=551.29,1307.90");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=502.72,1351.97");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=551.00,1419.20");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=425.23,1447.86");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=361.45,1446.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=288.25,1508.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=255.26,1416.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=170.91,1372.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=134.49,1480.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=85.43,1530.30");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=35.09,1599.81");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=90.28,1653.46");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=156.37,1694.89");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=84.49,1751.70");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=153.58,1755.36");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=131.60,1824.44");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=149.88,1943.92");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=207.78,2021.24");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=274.35,2020.22");
    }

}

