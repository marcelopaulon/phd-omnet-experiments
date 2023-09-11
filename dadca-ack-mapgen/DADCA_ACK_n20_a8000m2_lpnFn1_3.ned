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

network DADCA_ACK_n20_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=144.35,181.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=212.04,129.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=300.97,173.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=157.58,27.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=27.88,56.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=42.57,185.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=72.52,246.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=141.41,267.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=56.89,360.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=91.20,309.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=272.56,111.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=234.50,38.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=347.03,281.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=222.95,261.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=123.26,385.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=173.00,436.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=170.71,342.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=237.79,389.27");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=297.12,324.68");
    }

}

