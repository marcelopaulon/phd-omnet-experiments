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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=49.91,36.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=178.56,101.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=237.42,53.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=194.69,168.21");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=319.19,44.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=298.42,177.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=356.21,98.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=435.85,143.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=382.69,202.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=429.09,65.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=517.33,46.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=413.36,271.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=485.48,259.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=509.53,157.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=580.97,117.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=540.24,212.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=434.31,337.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=281.40,96.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=248.48,217.44");
    }

}

