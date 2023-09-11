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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=186.32,102.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=263.04,146.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=262.32,60.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=318.09,203.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=226.20,202.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=204.66,39.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=343.97,123.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=462.88,144.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=425.17,278.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=505.73,254.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=587.35,281.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=585.98,346.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=498.08,348.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=550.81,189.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=589.29,137.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=413.86,182.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=503.86,51.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=581.72,44.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=423.17,95.81");
    }

}

