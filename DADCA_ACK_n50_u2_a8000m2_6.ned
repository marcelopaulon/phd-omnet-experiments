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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=701.84,7753.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1203.18,7914.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1676.44,7979.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2112.33,7980.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3102.46,7843.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4287.37,7983.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5091.92,7979.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=62.59,7603.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1728.67,7612.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2107.55,7579.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2789.79,7685.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3567.71,7539.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3944.68,7580.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4558.55,7603.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5026.79,7603.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5671.97,7627.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=853.86,7409.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1227.96,7336.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2470.54,7459.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3236.87,7313.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4282.65,7308.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=956.43,7041.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1839.29,7218.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2814.28,7237.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3968.89,7124.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4644.67,7243.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4993.90,7189.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5376.14,7072.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5898.11,7115.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=270.74,6943.83");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1246.23,6771.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1864.71,6771.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2232.30,6868.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3313.33,6910.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5111.24,6765.55");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=215.16,6597.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2562.60,6664.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3412.60,6515.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3763.38,6675.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4374.22,6698.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=546.99,6470.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1542.72,6315.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2311.76,6400.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5588.10,6322.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=144.68,6216.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1890.02,6137.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2415.01,6032.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2892.56,6063.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3303.76,6131.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3896.24,6183.26");
    }

}

