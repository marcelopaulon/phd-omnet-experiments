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
        @display("i=misc/sensor,p=171.57,162.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=182.64,96.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=127.05,38.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=151.42,221.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=177.04,296.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=136.02,352.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=220.34,235.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=280.59,207.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=171.60,413.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=248.96,370.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=327.59,339.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=315.64,279.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=365.29,243.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=299.96,410.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=96.43,173.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=32.46,169.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=39.08,89.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=71.10,14.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=295.03,49.07");
    }

}

