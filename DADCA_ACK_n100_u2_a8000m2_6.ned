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
        @display("i=misc/sensor;p=378.48,7825.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2103.78,7824.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2444.41,7947.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3760.39,7781.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4882.80,7911.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=736.14,7751.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1537.74,7537.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2604.83,7551.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2958.04,7713.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5229.10,7549.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5639.16,7696.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=916.86,7448.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1825.50,7309.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2193.32,7363.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3266.93,7382.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3911.26,7266.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4262.94,7504.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4587.94,7280.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=51.49,7243.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=477.72,7171.76");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=886.29,7065.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2509.09,7178.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2897.85,7103.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5830.61,7244.42");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1194.15,6834.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2248.99,6797.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3295.54,6830.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3798.78,6890.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4478.13,6843.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5603.48,6947.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5998.77,6925.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=88.10,6562.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=711.58,6701.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1562.23,6676.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2966.56,6664.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3630.58,6540.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5002.31,6740.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=470.08,6374.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=823.07,6358.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3996.40,6491.18");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5502.28,6465.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5971.21,6349.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1592.80,6241.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2188.51,6257.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3250.17,6265.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3607.84,6099.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4553.58,6243.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5032.24,6075.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=512.32,5911.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1688.22,5888.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2807.26,5967.61");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3237.16,5821.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4245.15,5841.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5476.53,5839.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5893.26,5931.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=63.07,5704.62");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1117.46,5756.46");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2539.52,5665.17");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3829.17,5719.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4686.52,5697.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=296.81,5379.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=680.90,5491.36");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1697.51,5409.60");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2153.87,5465.69");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5239.21,5453.27");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=787.09,5087.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1320.05,5147.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2926.03,5239.00");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3590.79,5218.99");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3937.04,5154.95");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4335.34,5266.20");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=232.73,4932.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2119.11,4916.07");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2631.96,4835.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3192.59,4980.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4727.48,5021.43");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5481.41,4993.89");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5880.42,4801.16");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=521.81,4718.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1114.52,4617.39");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3649.26,4649.41");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4098.68,4646.72");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5443.30,4632.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=198.54,4507.09");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1362.93,4310.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1763.82,4507.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2396.37,4440.80");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3105.65,4388.40");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4634.71,4418.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5042.05,4499.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5940.61,4296.84");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=353.24,4073.80");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2010.12,4154.69");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2906.61,4094.43");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3451.68,4124.76");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3942.31,4197.91");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=902.00,3876.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1693.23,3914.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4506.99,4029.36");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4956.62,3802.83");
    }

}

