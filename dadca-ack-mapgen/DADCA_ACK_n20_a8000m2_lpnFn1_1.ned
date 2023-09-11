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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=33.69,129.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=71.92,205.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=163.66,223.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=218.89,187.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=288.36,230.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=163.83,136.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=210.25,78.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=224.18,4.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=166.04,23.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=82.68,31.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=282.80,115.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=345.98,41.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=421.17,27.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=460.11,89.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=504.58,46.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=385.22,149.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=394.62,209.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=389.48,288.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=336.10,324.60");
    }

}

