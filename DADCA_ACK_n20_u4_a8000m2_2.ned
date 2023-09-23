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

network DADCA_ACK_n20_u4_a8000m2_2
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
        @display("i=misc/sensor;p=757.48,7929.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1712.23,7911.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2170.28,7816.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3695.72,7788.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5238.16,7939.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=399.81,7747.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1426.62,7630.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3264.66,7663.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4324.72,7734.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4889.71,7596.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5753.00,7703.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=44.33,7385.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=547.04,7335.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1050.41,7405.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2444.73,7490.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5325.52,7276.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5730.31,7341.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=698.98,7018.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1654.82,7117.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2062.27,7065.95");
    }

}

