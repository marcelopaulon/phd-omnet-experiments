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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor;p=130.78,153.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=117.14,226.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=39.38,282.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1.67,189.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=56.14,350.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=114.50,327.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=206.78,308.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=46.51,411.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=48.80,152.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=195.42,126.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=169.61,37.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=230.93,64.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=290.67,41.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=309.02,105.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=20.51,55.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=101.86,32.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=161.50,378.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=226.10,365.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=217.20,433.44");
    }

}

