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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=154.28,7847.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=558.89,7986.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1082.33,7895.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2147.36,7876.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2736.25,7767.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4170.13,7906.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5374.93,7997.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5818.97,7789.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1722.90,7567.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2209.36,7522.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2986.82,7509.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3454.98,7670.12");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4579.74,7549.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4930.37,7729.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5384.21,7570.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=243.79,7348.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=617.28,7484.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1284.79,7388.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2624.67,7430.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3324.71,7290.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3859.37,7450.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5055.39,7380.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=950.39,7201.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1762.05,7015.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2851.61,7158.55");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4213.10,7095.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4571.35,7020.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5549.88,7250.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5913.26,7114.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=595.91,6817.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1351.54,6788.68");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3654.12,6813.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1054.21,6601.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1918.85,6651.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2264.97,6733.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4720.39,6563.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5313.84,6639.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5821.32,6739.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1622.41,6365.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2522.49,6464.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3237.41,6352.67");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3751.60,6445.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4100.16,6495.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=627.75,6033.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1971.84,6171.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2826.77,6227.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3731.17,6030.22");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4987.56,6040.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5587.34,6178.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5972.37,6255.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=125.49,6011.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1050.43,5940.84");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1597.59,5954.93");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2284.65,5834.84");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3177.46,5888.62");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4441.56,5952.18");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5341.24,5845.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5829.75,5896.15");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1053.96,5552.78");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1959.32,5631.31");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2804.00,5564.48");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4108.26,5700.94");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4561.18,5551.29");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=208.71,5332.22");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=685.80,5405.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1411.52,5443.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3678.96,5460.48");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5130.04,5461.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5815.04,5398.01");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=437.52,5030.43");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1194.20,5167.76");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2123.07,5248.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2500.56,5149.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2957.89,5115.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4258.20,5098.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=90.96,4804.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1002.81,4803.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1673.33,5004.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3491.07,4944.18");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4756.53,4983.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5533.24,4828.55");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5903.17,4857.61");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=388.32,4616.07");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1383.75,4761.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1691.06,4548.90");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2035.60,4641.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2640.15,4778.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3637.02,4582.54");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3994.77,4606.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4446.98,4753.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5115.70,4733.55");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=790.85,4475.37");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3357.00,4333.27");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4355.20,4413.23");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5369.46,4443.94");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5708.58,4306.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=259.49,4164.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1127.30,4169.26");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2305.72,4238.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2820.89,4234.49");
    }

}

