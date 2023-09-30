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

network DADCA_ACK_n20_u16_a8000m2_4
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
        @display("i=misc/sensor;p=253.28,7768.06");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1767.36,7785.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3706.37,7869.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4069.74,7955.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4726.75,7895.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5227.49,7968.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5673.98,7877.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2339.61,7691.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2947.76,7565.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3351.47,7668.58");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5636.24,7512.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=10.93,7399.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=655.96,7488.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1115.60,7492.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2528.52,7303.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3461.67,7290.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3945.29,7357.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4560.39,7496.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5162.11,7493.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=975.47,7169.91");
    }

}

