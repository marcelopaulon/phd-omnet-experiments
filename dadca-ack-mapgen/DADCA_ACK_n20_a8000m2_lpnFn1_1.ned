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
        @display("i=misc/sensor,p=89.50,191.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=49.16,252.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=20.33,334.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=110.64,364.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=169.65,328.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=221.93,370.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=240.26,284.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=330.78,272.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=304.23,216.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=365.58,345.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=447.97,352.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=446.23,281.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=509.39,235.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=477.38,148.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=439.80,200.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=414.74,121.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=576.45,118.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=640.01,116.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=608.12,51.10");
    }

}

