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

network DADCA_ACK_n100_u2_a8000m2_1
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
        @display("i=misc/sensor;p=1171.11,7820.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1713.26,7993.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2156.74,7939.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2874.87,7851.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4050.06,7859.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5239.26,7879.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5731.08,7848.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=59.58,7735.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=696.35,7595.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1589.74,7572.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1987.25,7509.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2437.43,7512.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3130.58,7529.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3643.35,7554.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4649.48,7746.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=321.92,7431.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1025.68,7275.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4249.75,7281.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4878.62,7400.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5473.05,7412.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5869.13,7459.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1526.06,7155.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1942.51,7113.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2616.99,7126.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3389.56,7035.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5802.81,7017.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=796.68,6940.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2296.01,6944.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3004.20,6836.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3965.45,6821.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4569.55,6936.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4995.44,6967.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=213.26,6549.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=564.75,6623.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1441.26,6756.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1993.62,6749.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3498.18,6654.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4298.73,6649.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4826.06,6541.06");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5412.77,6652.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5933.57,6677.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=880.94,6449.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1379.23,6371.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1727.66,6504.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2157.02,6304.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2747.67,6425.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=457.38,6201.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=825.46,6065.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1697.20,6075.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3269.63,6085.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3761.18,6132.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4172.75,6048.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4599.37,6099.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5646.49,6255.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=111.93,5931.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2055.43,5775.58");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5184.26,5910.27");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5536.74,5790.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5892.69,5958.64");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=453.64,5688.05");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1363.82,5688.49");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1715.27,5652.92");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3013.37,5670.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3903.69,5655.70");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4713.55,5573.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=185.50,5294.32");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=929.85,5316.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2163.73,5423.00");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5247.09,5343.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5663.48,5347.48");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=595.57,5149.34");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1760.26,5226.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2580.17,5249.54");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3385.78,5228.75");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3916.78,5163.56");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4342.19,5145.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4861.99,5114.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=51.49,4842.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1185.73,5013.71");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2031.57,4925.08");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2970.74,5016.27");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3821.99,4791.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5142.53,4831.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5552.34,4986.30");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5963.09,4902.28");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=801.60,4607.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1562.94,4689.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2492.61,4644.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2982.36,4661.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4191.17,4658.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4672.32,4588.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=235.60,4399.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2171.93,4467.74");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3093.70,4290.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4242.69,4288.90");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4986.00,4383.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5374.33,4486.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5754.21,4389.48");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1297.73,4184.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1802.14,4281.84");
    }

}

