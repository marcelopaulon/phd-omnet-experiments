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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=108.89,165.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=55.69,228.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=119.90,314.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=56.53,351.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=117.85,377.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=49.07,446.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=46.17,528.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=95.68,587.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=172.11,555.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=188.73,632.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=174.55,483.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=50.23,656.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=235.74,413.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=308.70,474.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=245.13,520.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=277.91,366.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=268.80,306.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=300.52,641.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=377.01,646.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=471.15,650.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=499.24,726.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=551.77,758.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=578.62,862.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=569.47,952.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=661.55,946.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=683.26,1029.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=629.70,1111.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=711.53,1136.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=765.40,987.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=803.19,1115.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=602.96,1019.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=440.11,972.61");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=379.76,944.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=405.75,874.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=302.66,1007.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=335.19,1073.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=358.88,1170.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=278.52,1227.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=416.47,1200.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=457.56,1245.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=519.53,1252.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=502.74,1194.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=453.84,1136.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=545.89,1075.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=432.06,1064.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=380.93,810.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=309.08,847.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=209.42,844.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=261.83,923.84");
    }

}

