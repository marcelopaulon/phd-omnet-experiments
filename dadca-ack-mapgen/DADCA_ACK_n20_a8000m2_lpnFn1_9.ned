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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=170.49,127.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=228.85,98.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=169.84,35.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=103.87,189.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=32.37,188.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=20.85,96.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=71.33,18.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=208.52,199.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=141.38,241.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=96.52,329.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=61.31,267.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=188.53,344.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=127.29,421.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=18.27,324.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=245.70,409.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=339.87,159.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=200.66,270.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=268.32,255.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=345.98,245.53");
    }

}

