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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=376.78,7765.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1605.30,7818.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4205.81,7771.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4550.05,7950.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4940.18,7853.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1041.47,7641.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2108.78,7673.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2708.92,7635.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3056.55,7744.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3428.32,7659.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4543.43,7560.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=602.02,7489.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2446.81,7399.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3922.76,7287.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4888.20,7365.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5230.25,7289.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5649.01,7298.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=25.91,7250.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=476.83,7098.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1709.42,7152.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2145.85,7182.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3007.03,7152.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1104.42,6981.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2596.86,6970.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3375.59,6987.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4149.14,6870.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4652.38,6973.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5480.00,6799.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1050.36,6625.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1743.50,6708.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2096.15,6659.29");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2490.41,6618.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2969.90,6680.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3351.11,6585.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3812.25,6745.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4544.77,6598.25");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5128.65,6708.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=400.34,6467.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1627.41,6370.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2037.47,6294.38");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3846.32,6307.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5605.78,6458.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=805.13,6049.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1231.28,6024.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1747.19,6039.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2385.56,6251.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2874.73,6212.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3461.73,6175.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5162.40,6114.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=408.71,5863.71");
    }

}

