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
        @display("i=misc/sensor;p=82.84,7924.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=518.05,7925.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1399.37,7805.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1858.44,7776.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2257.03,7890.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3372.71,7974.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4732.78,7812.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5205.62,7965.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1059.14,7579.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2777.87,7737.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3825.24,7611.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4334.02,7699.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5807.25,7693.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=74.14,7329.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1614.03,7349.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2202.09,7456.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4538.87,7285.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5164.28,7475.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5879.62,7311.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=526.27,7237.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1322.75,7127.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2658.22,7136.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3003.46,7033.91");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3313.21,7250.16");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3904.77,7032.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4821.18,7011.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5441.12,7101.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=98.11,6863.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=687.86,6870.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5809.88,6943.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1501.32,6692.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2077.12,6713.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2749.86,6739.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3210.66,6710.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3563.92,6647.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4320.02,6589.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4786.52,6528.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5143.87,6660.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=137.58,6468.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=764.58,6427.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2544.05,6413.23");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2932.46,6286.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5920.52,6482.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=153.57,6022.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=885.69,6082.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1238.23,6229.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1634.15,6240.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2303.27,6156.14");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3273.53,6058.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4252.54,6185.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4843.06,6180.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1481.93,5812.49");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2144.46,5780.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2821.12,5881.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3541.35,5811.27");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5400.82,5984.78");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5971.23,5912.00");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=158.02,5602.61");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=696.30,5593.28");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1800.62,5604.96");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3067.99,5571.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3937.72,5613.24");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5187.60,5611.84");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1126.14,5432.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2388.10,5400.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3460.55,5281.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4354.90,5522.07");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4788.45,5314.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1481.26,5246.95");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3805.53,5144.86");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4273.01,5162.19");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5095.24,5140.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5479.86,5191.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=6.53,4992.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=417.62,4888.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=814.61,4891.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1248.70,4856.83");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2114.08,4999.72");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2563.70,4786.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3160.73,4865.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4603.58,4895.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5801.41,4961.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=172.65,4635.36");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1676.82,4736.89");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3506.08,4746.26");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5070.74,4692.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5509.58,4759.53");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5904.45,4544.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=951.65,4394.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1362.87,4498.14");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2067.03,4519.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3621.55,4404.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3961.64,4517.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4441.05,4338.81");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=269.69,4189.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=692.20,4123.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1589.92,4111.57");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2314.42,4220.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2955.99,4256.41");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4129.36,4131.44");
    }

}

