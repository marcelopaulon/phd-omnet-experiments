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
        @display("i=misc/sensor;p=1535.31,7987.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2368.56,7839.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2719.85,7973.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3826.03,7789.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4465.51,7766.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4965.27,7866.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5547.48,7875.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=439.68,7664.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=865.20,7663.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1220.07,7568.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1711.83,7615.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2795.76,7544.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3292.36,7581.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4077.17,7533.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5274.37,7584.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2094.73,7488.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2546.08,7266.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4958.42,7275.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5780.59,7481.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=300.18,7113.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1637.44,7021.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2953.43,7213.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3592.29,7157.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4262.18,7117.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4624.15,7150.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5375.55,7080.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=21.93,6874.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=952.94,6992.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2395.51,6866.10");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2937.97,6809.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4849.88,6846.14");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5785.01,6867.25");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=616.25,6561.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1124.50,6675.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2083.22,6596.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3975.26,6700.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4518.79,6730.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5389.83,6616.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=19.08,6338.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1600.27,6466.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2586.98,6325.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3465.51,6437.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4699.83,6306.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5160.48,6331.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5709.62,6349.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1044.45,6147.18");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1500.80,6104.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1922.95,6212.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3122.15,6267.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3875.84,6157.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4275.08,6124.36");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5977.90,6064.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=711.88,5955.62");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2540.76,5875.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3100.56,5881.09");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4801.80,5968.90");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5166.08,5926.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=179.52,5744.23");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1581.94,5755.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3860.77,5577.93");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4285.05,5552.69");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5207.02,5532.26");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5782.92,5721.63");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=564.78,5395.02");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1035.52,5459.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2071.55,5413.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2454.09,5386.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2990.50,5398.26");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4652.74,5389.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5780.79,5281.12");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1623.67,5251.47");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4027.58,5055.11");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4376.04,5155.07");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5258.94,5054.33");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=111.78,5029.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=898.60,4823.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1415.64,4878.71");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1900.42,4969.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2994.36,4814.76");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3419.55,4907.34");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=547.77,4728.61");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2195.90,4774.60");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4137.06,4616.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4471.19,4737.91");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4837.42,4712.22");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5447.64,4705.67");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5924.46,4724.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=304.84,4465.59");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1505.12,4496.11");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2088.62,4368.23");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3022.05,4300.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3582.36,4494.61");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5418.16,4303.82");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5904.51,4373.02");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=430.74,4046.49");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1131.27,4247.17");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1754.96,4155.07");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2359.62,4129.89");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3642.99,4118.52");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4073.68,4177.56");
    }

}

