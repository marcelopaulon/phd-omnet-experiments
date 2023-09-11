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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=138.31,185.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=39.66,191.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=58.51,251.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=143.91,297.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=222.63,252.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=40.45,47.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=99.26,17.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=233.27,96.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=177.35,138.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=153.10,372.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=56.69,372.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=164.72,74.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=208.81,6.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=273.99,143.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=289.02,205.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=339.98,243.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=297.21,303.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=231.51,346.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=209.01,411.21");
    }

}

