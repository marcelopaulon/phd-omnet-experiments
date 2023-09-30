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
        @display("i=misc/sensor,p=93.68,38.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=20.01,72.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=37.60,154.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=103.67,161.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=190.94,119.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=240.63,40.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=159.82,32.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=299.75,99.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=367.06,157.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=291.72,163.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=469.48,171.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=522.51,136.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=466.87,236.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=485.95,293.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=516.74,357.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=529.02,448.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=455.51,515.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=527.31,566.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=564.22,638.91");
    }

}

