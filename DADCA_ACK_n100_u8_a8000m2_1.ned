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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=1083.53,7962.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1503.97,7939.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1926.95,7993.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2850.49,7774.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3229.14,7996.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5733.81,7984.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=247.00,7660.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1056.89,7537.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1454.76,7514.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1822.48,7598.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2508.50,7613.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3652.09,7635.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4421.38,7719.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5231.30,7741.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=594.99,7436.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3262.61,7412.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4909.99,7289.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5432.35,7413.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=121.51,7048.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=773.11,7087.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1136.46,7018.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1477.53,7130.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2178.59,7256.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4100.79,7243.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4450.89,7046.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1872.63,6945.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2461.15,6825.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2825.12,7004.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3480.73,6943.10");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5072.83,6955.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5705.84,6805.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=492.18,6582.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1006.29,6639.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1508.62,6528.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4058.06,6552.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4762.16,6661.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5359.10,6695.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1995.17,6483.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2349.35,6299.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2996.86,6507.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3587.05,6301.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4356.61,6367.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5049.86,6440.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5653.53,6426.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=502.98,6042.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=943.22,6260.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1389.81,6065.75");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2721.32,6246.99");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3087.15,6130.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4819.87,6085.08");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5326.99,6184.55");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5996.13,6210.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1667.98,5774.87");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2648.95,5836.83");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3948.98,5872.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4316.42,5795.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5138.49,5836.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5770.55,5899.06");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2146.12,5730.12");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3094.07,5708.23");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3498.36,5575.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=340.73,5355.90");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=764.81,5508.62");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1413.10,5492.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2338.35,5421.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3137.64,5285.64");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3821.78,5359.23");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4456.29,5415.94");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5259.39,5456.58");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5919.46,5329.19");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=68.31,5068.07");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1789.51,5172.11");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2724.61,5141.69");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4842.20,5200.46");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5571.86,5190.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=764.03,4972.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1294.86,4949.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2339.54,4911.27");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3693.48,4808.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4258.25,4988.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5882.34,4822.68");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=240.52,4611.90");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=725.75,4577.51");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1118.98,4608.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1644.97,4720.31");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3001.26,4615.13");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4193.22,4582.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4604.18,4573.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5192.06,4762.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2118.28,4323.48");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2459.51,4474.17");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5494.84,4437.49");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=545.16,4169.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=914.97,4133.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1266.26,4207.20");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1637.74,4148.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2434.79,4090.95");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3319.28,4051.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3908.68,4266.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4871.98,4148.07");
    }

}

