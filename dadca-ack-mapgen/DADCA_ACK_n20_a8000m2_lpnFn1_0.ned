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
        @display("i=misc/sensor,p=148.96,38.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=80.89,33.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=155.48,140.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=205.41,98.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=216.98,208.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=250.54,156.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=342.45,158.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=402.04,101.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=404.30,40.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=470.81,11.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=308.42,96.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=147.39,210.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=28.20,144.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=2.29,229.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=29.02,318.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=29.42,0.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=39.53,83.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=315.09,246.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=338.40,326.84");
    }

}

