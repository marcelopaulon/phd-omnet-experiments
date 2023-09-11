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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=12.79,92.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=28.33,30.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=157.56,68.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=222.65,85.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=210.19,161.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=93.11,36.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=211.18,260.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=134.09,203.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=194.92,358.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=112.62,381.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=30.04,328.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=23.98,248.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=300.72,179.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=269.85,273.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=69.50,177.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=353.50,113.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=398.14,158.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=485.96,146.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=542.70,226.23");
    }

}

