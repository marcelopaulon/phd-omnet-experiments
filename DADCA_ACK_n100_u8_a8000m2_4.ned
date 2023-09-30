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

network DADCA_ACK_n100_u8_a8000m2_4
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
        @display("i=misc/sensor;p=157.36,7850.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=696.69,7972.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1165.38,7855.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2200.93,7905.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3033.35,7901.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3441.67,7913.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4091.44,7904.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5778.29,7767.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=620.26,7594.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1582.10,7682.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3864.69,7614.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4383.79,7706.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5111.85,7705.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=10.71,7498.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1018.83,7483.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1723.46,7271.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2147.55,7425.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2831.78,7267.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3374.29,7482.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3673.32,7275.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4257.80,7332.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4711.77,7273.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5567.55,7363.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=71.66,7103.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1069.90,7071.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3241.51,7126.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5149.92,7143.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5873.38,7173.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=443.39,6921.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1441.33,6923.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3766.51,6842.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4112.06,6974.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4493.18,6943.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4936.85,6832.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5596.12,6814.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=211.70,6569.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=888.78,6535.85");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2065.41,6705.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2550.98,6698.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3197.43,6648.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4251.59,6559.04");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4698.36,6538.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1373.02,6321.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1694.68,6473.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3471.39,6317.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3929.48,6363.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5188.10,6503.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=45.59,6239.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=422.20,6075.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1896.93,6154.84");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2316.50,6238.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2717.00,6251.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4618.83,6043.22");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5726.00,6161.33");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=839.18,5948.78");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1260.78,5981.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2243.27,5846.52");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2608.15,5806.33");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3150.77,6012.44");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4023.26,5883.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5030.49,5915.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=318.55,5616.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1820.20,5757.43");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3252.55,5658.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3630.42,5690.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4483.12,5663.68");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5527.70,5570.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5827.55,5755.48");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=907.63,5509.72");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1325.55,5460.61");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3160.42,5284.46");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4924.73,5480.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5827.71,5323.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=20.09,5068.55");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1030.56,5155.39");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2409.14,5098.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2757.14,5197.30");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4261.07,5274.69");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4616.50,5202.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5512.59,5067.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=466.40,5020.38");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1364.56,4784.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1906.73,4985.62");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2849.30,4841.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3409.76,4834.97");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3732.11,4992.70");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5163.83,4842.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=661.99,4538.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=937.89,4763.57");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2071.80,4665.76");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4033.44,4773.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4563.60,4738.73");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5718.62,4702.47");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1176.75,4306.65");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1688.54,4330.59");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2395.21,4483.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3445.29,4392.18");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5405.78,4474.53");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=305.90,4106.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=778.17,4109.33");
    }

}

