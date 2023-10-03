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

network DADCA_ACK_n20_u3_a8000m2_3
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
        @display("i=misc/sensor;p=109.57,7905.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3786.29,7960.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5004.38,7862.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5734.90,7899.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=263.71,7575.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=780.30,7603.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1222.14,7746.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1686.63,7731.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2079.66,7668.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2440.47,7682.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3002.38,7649.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3509.22,7679.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3980.98,7649.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4497.19,7659.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=575.39,7263.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3236.75,7297.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4779.48,7351.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5338.23,7419.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5885.42,7484.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=62.47,7254.30");
    }

}

